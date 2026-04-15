from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Optional

import rclpy.time
import tf2_ros

import ros_system_monitor as rsm
from sensor_msgs.msg import NavSatFix
from spark_config.config import Config, register_config


def latlon_to_utm(lat, lon):
    """Convert WGS84 lat/lon (degrees) to UTM easting/northing (meters)."""
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    e_prime2 = e2 / (1 - e2)
    k0 = 0.9996

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    zone_number = int((lon + 180) / 6) + 1
    lon0 = math.radians((zone_number - 1) * 6 - 180 + 3)

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    tan_lat = math.tan(lat_rad)

    N = a / math.sqrt(1 - e2 * sin_lat**2)
    T = tan_lat**2
    C = e_prime2 * cos_lat**2
    A = cos_lat * (lon_rad - lon0)

    M = a * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat_rad
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat_rad)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat_rad)
        - (35 * e2**3 / 3072) * math.sin(6 * lat_rad)
    )

    easting = k0 * N * (
        A
        + (1 - T + C) * A**3 / 6
        + (5 - 18 * T + T**2 + 72 * C - 58 * e_prime2) * A**5 / 120
    ) + 500000.0

    northing = k0 * (
        M
        + N
        * tan_lat
        * (
            A**2 / 2
            + (5 - T + 9 * C + 4 * C**2) * A**4 / 24
            + (61 - 58 * T + T**2 + 600 * C - 330 * e_prime2) * A**6 / 720
        )
    )
    if lat < 0:
        northing += 10000000.0

    return easting, northing, zone_number


class RelocalizationMonitor:
    def __init__(
        self,
        config: RelocalizationMonitorConfig,
        nickname: str,
        monitor_name: Optional[str] = None,
    ):
        self.nickname = nickname
        self.warning_threshold = config.warning_threshold
        self.timeout = config.timeout
        self.robot_name = monitor_name
        self.last_fix_time = None

        self.origin_easting = None
        self.origin_northing = None
        origin_str = os.environ.get("ADT4_ORIGIN_UTM")
        if origin_str:
            try:
                parts = origin_str.split(",")
                self.origin_easting = float(parts[0])
                self.origin_northing = float(parts[1])
            except (ValueError, IndexError):
                pass

    def register_monitor(self, monitor):
        self.monitor_callback = monitor.update_node_info
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, monitor)
        self.gps_frame = f"{self.robot_name}/gps"
        self.sub = monitor.create_subscription(NavSatFix, "fix", self._gps_cb, 10)
        self.timer = monitor.create_timer(1.0, self._timer_cb)

    def _report(self, status, note):
        info = rsm.TrackedNodeInfo(
            nickname=self.nickname,
            last_heartbeat=-1,
            node_name="<external>",
            status=status,
            notes=note,
        )
        self.monitor_callback(info)

    def _gps_cb(self, msg: NavSatFix):
        self.last_fix_time = time.monotonic()

        if self.origin_easting is None:
            return

        gps_easting, gps_northing, _ = latlon_to_utm(msg.latitude, msg.longitude)

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.gps_frame, rclpy.time.Time.from_msg(msg.header.stamp)
            )
        except tf2_ros.TransformException:
            self._report(rsm.Status.WARNING, "TF map->gps not available")
            return

        est_easting = tf.transform.translation.x + self.origin_easting
        est_northing = tf.transform.translation.y + self.origin_northing

        dist = math.hypot(gps_easting - est_easting, gps_northing - est_northing)

        if dist < self.warning_threshold:
            self._report(rsm.Status.NOMINAL, f"Relocalization error: {dist:.2f} m")
        else:
            self._report(rsm.Status.WARNING, f"Relocalization error: {dist:.2f} m")

    def _timer_cb(self):
        if self.origin_easting is None:
            self._report(
                rsm.Status.ERROR, "ADT4_ORIGIN_UTM not set or invalid"
            )
            return
        if self.last_fix_time is None:
            self._report(rsm.Status.STARTUP, "Waiting for first GPS fix")
            return
        dt = time.monotonic() - self.last_fix_time
        if dt > self.timeout:
            self._report(
                rsm.Status.ERROR,
                f"No GPS fix [Δt={dt:.1f}s since last msg]",
            )


@register_config(
    "external_monitor",
    name="RelocalizationMonitor",
    constructor=RelocalizationMonitor,
)
@dataclass
class RelocalizationMonitorConfig(Config):
    warning_threshold: float = 5.0
    timeout: float = 10.0
