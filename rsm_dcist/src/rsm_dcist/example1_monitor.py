from __future__ import annotations

import functools
from dataclasses import dataclass

import ros_system_monitor as rsm
from spark_config.config import Config, register_config
from std_msgs.msg import String


class Example1NodeMonitor:
    def __init__(self, config: Example1NodeMonitorConfig, nickname: str):
        self.nickname = nickname
        self.divider = config.divider
        self.mod = config.mod
        self.monitor = None

    def register_monitor(self, monitor):
        self.monitor_callback = functools.partial(
            monitor.update_node_info, self.nickname
        )
        self.sub = monitor.create_subscription(
            String, "~/example1_node_topic", self.callback, 1
        )

    def callback(self, msg):
        msg_idx = int(msg.data.split(" ")[3])
        if msg_idx // self.divider % self.mod:
            status = rsm.Status.NOMINAL
        else:
            status = rsm.Status.ERROR

        note = f"Current messaged idx {msg_idx}"
        self.monitor_callback("<external>", status, note)


@register_config(
    "external_monitor", name="Example1NodeMonitor", constructor=Example1NodeMonitor
)
@dataclass
class Example1NodeMonitorConfig(Config):
    divider: int = None
    mod: int = None
