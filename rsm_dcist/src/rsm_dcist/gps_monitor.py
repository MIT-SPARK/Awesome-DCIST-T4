from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import time

import ros_system_monitor as rsm
from spark_config.config import Config, register_config
from sensor_msgs.msg import NavSatFix


class GpsMonitor:
    def __init__(
        self,
        config: GpsMonitorConfig,
        nickname: str,
        monitor_name: Optional[str] = None,
    ):
        self.nickname = nickname
        self.timeout = config.timeout
        self.last_fix_time = None
        self.status = rsm.Status.STARTUP

    def register_monitor(self, monitor):
        self.monitor_callback = monitor.update_node_info
        self.sub = monitor.create_subscription(NavSatFix, "fix", self._gps_cb, 10)
        self.timer = monitor.create_timer(1.0, self._timer_cb)

    def _gps_status(self, msg: NavSatFix):
        s = msg.status.status
        if s == -1:
            return rsm.Status.ERROR, "No Fix"
        elif s == 0:
            return rsm.Status.WARNING, "Fix (No RTK)"
        elif s == 2:
            return rsm.Status.NOMINAL, "RTK Fix"
        return rsm.Status.ERROR, "Unknown"

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
        self.status, label = self._gps_status(msg)
        cov = msg.position_covariance
        sx = cov[0] ** 0.5
        sy = cov[4] ** 0.5
        sz = cov[8] ** 0.5
        self._report(self.status, f"{label} [σx={sx:.2f}, σy={sy:.2f}, σz={sz:.2f}]")

    def _timer_cb(self):
        if self.last_fix_time is None:
            self._report(rsm.Status.STARTUP, "Waiting for first GPS fix")
            return
        dt = time.monotonic() - self.last_fix_time
        if dt > self.timeout:
            self._report(rsm.Status.ERROR, f"No GPS fix [Δt={dt:.1f}s since last msg]")


@register_config("external_monitor", name="GpsMonitor", constructor=GpsMonitor)
@dataclass
class GpsMonitorConfig(Config):
    timeout: float = 10.0
