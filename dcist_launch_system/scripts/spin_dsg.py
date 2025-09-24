#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math
import time
from visualization_msgs.msg import Marker

class CircularCameraNode(Node):
    def __init__(self):
        super().__init__('circular_camera_node')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Parameters
        self.rate_hz = 30
        self.radius = 20  # distance from map center
        self.height = 2.0  # height of the camera
        self.angular_speed = 0.2  # radians per second
        self.xc = -8
        self.yc = -15

        self.frame_id = 'map'
        self.child_frame_id = 'camera_frame'

        #self.frame_id = 'camera_frame'
        #self.child_frame_id = 'map'

        self.marker_pub = self.create_publisher(Marker, 'orbit_center_marker', 10)
        
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)

    def timer_callback(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9  # seconds
        angle = self.angular_speed * t
        
        x = self.radius * math.cos(angle)
        y = self.radius * math.sin(angle)
        z = self.height

        # Compute rotation so camera faces the origin
        yaw = math.atan2(-y, -x)  # look at origin
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)

        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = self.frame_id
        t_msg.child_frame_id = self.child_frame_id
        t_msg.transform.translation.x = x + self.xc
        t_msg.transform.translation.y = y + self.yc
        t_msg.transform.translation.z = z
        t_msg.transform.rotation.x = quat[0]
        t_msg.transform.rotation.y = quat[1]
        t_msg.transform.rotation.z = quat[2]
        t_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t_msg)

        # Marker message for the orbit center
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = t_msg.header.stamp
        marker.ns = "orbit_center"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.xc)
        marker.pose.position.y = float(self.yc)
        marker.pose.position.z = float(self.height)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)



def main(args=None):
    rclpy.init(args=args)
    node = CircularCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

