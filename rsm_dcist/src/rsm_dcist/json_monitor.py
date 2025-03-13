from __future__ import annotations

from dataclasses import dataclass
import json

import ros_system_monitor as rsm
from spark_config.config import Config, register_config
from std_msgs.msg import String


class JsonMonitor:
    def __init__(self, config: JsonMonitorConfig, nickname: str):
        self.nickname = nickname
        self.config = config

    @classmethod
    def load(cls, path):
        config = Config.load(JsonMonitorConfig, path)
        return cls(config)

    def set_callback(self, monitor, monitor_callback):
        self._update_cb = monitor_callback
        self.sub = monitor.create_subscription(String, "~/status", self._callback, 1)

    def _callback(self, msg):
        try:
            contents = json.loads(msg.data)
        except Exception as e:
            self._update_cb("???", rsm.Status.ERROR, f"invalid json: {e}")

        if "status" not in contents:
            self._update_cb("???", rsm.Status.ERROR, f"status missing: '{contents}'")

        if self.config.nickname != "":
            if "nickname" not in contents:
                self._update_cb(
                    "???", rsm.Status.ERROR, f"nickname missing: '{contents}'"
                )

            curr_name = contents["nickname"]
            if self.config.nickname != contents["nickname"]:
                self._update_cb(
                    "???",
                    rsm.Status.ERROR,
                    f"name mismatch: got '{curr_name} vs. '{self.config.nickname}'",
                )

        status, status_str = rsm.str_to_status(contents["status"])
        note = contents.get("note", "")
        if status_str is not None:
            note += f" ({status_str})"

        self._update_cb(contents.get("node_name", "???"), status, note)


@register_config("json_monitor", name="JsonMonitor", constructor=JsonMonitor)
@dataclass
class JsonMonitorConfig(Config):
    namespace: str = ""
    nickname: str = ""
