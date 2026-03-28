from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import ros_system_monitor as rsm
from rtcm_msgs.msg import Message as RTCMMessage
from spark_config.config import Config, register_config


class NtripMonitor:
    def __init__(
        self,
        config: NtripMonitorConfig,
        nickname: str,
        monitor_name: Optional[str] = None,
    ):
        self.nickname = nickname
        self.timeout = config.timeout
        self.last_recv_time = None
        self.status = rsm.Status.STARTUP

    def register_monitor(self, monitor):
        self.monitor_callback = monitor.update_node_info
        self.sub = monitor.create_subscription(RTCMMessage, "rtcm", self._ntrip_cb, 10)
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

    def _ntrip_cb(self, msg: RTCMMessage):
        self.last_recv_time = time.monotonic()
        self.status = rsm.Status.NOMINAL

    def _timer_cb(self):
        if self.last_recv_time is None:
            self._report(rsm.Status.STARTUP, "Waiting for first NTRIP message")
            return
        dt = time.monotonic() - self.last_recv_time
        if dt > self.timeout:
            self._report(
                rsm.Status.ERROR, f"No NTRIP message [Δt={dt:.1f}s since last msg]"
            )
        else:
            self._report(
                self.status, f"NTRIP message received [Δt={dt:.1f}s since last msg]"
            )


@register_config("external_monitor", name="NtripMonitor", constructor=NtripMonitor)
@dataclass
class NtripMonitorConfig(Config):
    timeout: float = 10.0
