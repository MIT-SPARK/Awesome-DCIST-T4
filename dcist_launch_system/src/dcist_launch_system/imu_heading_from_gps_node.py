#!/usr/bin/env python3
"""
gps_heading_node.py

Estimates the yaw offset between the odom frame and ENU (true north/east)
by comparing GPS displacement heading with odometry displacement heading.

Once calibrated, publishes a synthetic sensor_msgs/Imu with global yaw
so that navsat_transform_node can use it for the UTM↔odom rotation.

Requires the robot to move ~5 m before it locks on.
"""

import math

import rclpy
import tf_transformations
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix


# ── helpers ───────────────────────────────────────────────
def yaw_from_quat(q):
    _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def quat_from_yaw(yaw):
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    msg = Quaternion()
    msg.x, msg.y, msg.z, msg.w = q
    return msg


def normalize(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# rough meters-per-degree at mid latitudes
LAT_M_PER_RAD = 111_320.0 * 180 / math.pi


def lon_m(lat_rad):
    return LAT_M_PER_RAD * math.cos(lat_rad)


class GpsHeadingNode(Node):
    def __init__(self):
        super().__init__("gps_heading_node")

        self.declare_parameter("min_displacement_m", -1.0)
        self.declare_parameter("publish_rate", -1.0)
        self.declare_parameter("imu_frame", "")

        self.min_disp = self.get_parameter("min_displacement_m").value
        assert self.min_disp > 0, "You forgot to set min_displacement_m"
        rate = self.get_parameter("publish_rate").value
        assert rate > 0, "You forgot to set rate"
        self.imu_frame = self.get_parameter("imu_frame").value
        assert self.imu_frame != "", "You forgot to set imu_frame"

        # ── State ─────────────────────────────────────────
        self.calibrated = False
        self.yaw_offset = 0.0  # global_yaw = odom_yaw + yaw_offset

        # first GPS and odom readings (captured together)
        self.gps_anchor = None  # (lat_rad, lon_rad)
        self.odom_anchor = None  # (x, y)
        self.latest_odom_yaw = 0.0

        # ── Subscriptions ─────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.create_subscription(NavSatFix, "~/gps_in", self.gps_cb, sensor_qos)
        self.create_subscription(Odometry, "~/odom_in", self.odom_cb, 10)

        # ── Publisher ─────────────────────────────────────
        self.imu_pub = self.create_publisher(Imu, "~/fake_imu_out", 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_imu)

        self.get_logger().info(
            f"Waiting for {self.min_disp:.1f} m GPS displacement to "
            f"calibrate heading offset …"
        )

    # ── callbacks ─────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.latest_odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.latest_odom_x = msg.pose.pose.position.x
        self.latest_odom_y = msg.pose.pose.position.y

        # capture anchor on first odom
        if self.odom_anchor is None:
            self.odom_anchor = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

    def gps_cb(self, msg: NavSatFix):
        if self.calibrated:
            return
        if self.odom_anchor is None:
            return  # need odom first

        lat_r = math.radians(msg.latitude)
        lon_r = math.radians(msg.longitude)

        # capture anchor on first valid GPS
        if self.gps_anchor is None:
            self.gps_anchor = (lat_r, lon_r)
            self.get_logger().info(
                f"GPS anchor set: {msg.latitude:.7f}, {msg.longitude:.7f}"
            )
            return

        # displacement in metres (local tangent plane, ENU)
        dlat = lat_r - self.gps_anchor[0]
        dlon = lon_r - self.gps_anchor[1]
        de = dlon * lon_m(lat_r)  # east
        dn = dlat * LAT_M_PER_RAD  # north

        gps_dist = math.hypot(de, dn)
        if gps_dist < self.min_disp:
            self.get_logger().info(
                f"Current GPS dist: {gps_dist} ( < {self.min_disp} )"
            )
            return

        # GPS heading in ENU (atan2(north, east) is standard math angle)
        gps_heading = math.atan2(dn, de)  # ENU: 0=east, pi/2=north

        # Odom heading from displacement
        dx = self.latest_odom_x - self.odom_anchor[0]
        dy = self.latest_odom_y - self.odom_anchor[1]
        odom_heading = math.atan2(dy, dx)

        self.yaw_offset = normalize(gps_heading - odom_heading)
        self.calibrated = True

        self.get_logger().info(
            f"Heading calibrated!  "
            f"GPS heading:  {math.degrees(gps_heading):.1f}°  "
            f"Odom heading: {math.degrees(odom_heading):.1f}°  "
            f"Offset:       {math.degrees(self.yaw_offset):.1f}°"
        )

    # ── publisher ─────────────────────────────────────────
    def publish_imu(self):
        if not self.calibrated:
            return

        global_yaw = normalize(self.latest_odom_yaw + self.yaw_offset)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame

        msg.orientation = quat_from_yaw(global_yaw)

        # orientation covariance: only yaw is meaningful
        #  row-major 3×3 for (roll, pitch, yaw)
        msg.orientation_covariance = [
            0.01,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            # 0.02,  # yaw uncertainty from GPS-derived heading
            0.1,  # yaw uncertainty from GPS-derived heading
        ]

        # explicitly mark gyro/accel as unused
        msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
        msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8

        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = GpsHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
