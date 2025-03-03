from __future__ import annotations

from dataclasses import dataclass

from ros_system_monitor.node_info import NodeInfo, Status
from spark_config.config import Config, register_config
from std_msgs.msg import String


class Example1NodeMonitor:
    def __init__(self, config: Example1NodeMonitorConfig):
        self.divider = config.divider
        self.mod = config.mod
        self.monitor = None

    @classmethod
    def load(cls, path):
        config = Config.load(Example1NodeMonitorConfig, path)
        return cls(config)

    def register_callbacks(self, monitor):
        self.monitor = monitor
        subscriber = monitor.create_subscription(
            String, "~/example1_node_topic", self.callback, 1
        )
        monitor.add_subscribers([subscriber])

    def callback(self, msg):
        msg_idx = int(msg.data.split(" ")[3])
        if msg_idx // self.divider % self.mod:
            status = Status.NOMINAL
        else:
            status = Status.ERROR
        note = f"Current messaged idx {msg_idx}"
        info = NodeInfo(
            nickname="example1",
            node_name="<external monitor>",
            status=status,
            notes=note,
        )

        self.monitor.update_node_info(info)


@register_config(
    "external_monitor", name="Example1NodeMonitor", constructor=Example1NodeMonitor
)
@dataclass
class Example1NodeMonitorConfig(Config):
    divider: int = None
    mod: int = None
