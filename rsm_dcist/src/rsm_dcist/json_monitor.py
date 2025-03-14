from __future__ import annotations

from dataclasses import dataclass
import json
import functools

import ros_system_monitor as rsm
import spark_config as sc
from std_msgs.msg import String


class JsonMonitor:
    """External monitor that listens to json status report."""

    def __init__(self, config: JsonMonitorConfig, nickname: str):
        self.nickname = nickname

    def register_monitor(self, monitor):
        self._update_cb = functools.partial(monitor.update_node_info, self.nickname)
        topic = rsm.get_topic(self.nickname)
        self.sub = monitor.create_subscription(String, str(topic), self._callback, 1)

    def _callback(self, msg):
        try:
            contents = json.loads(msg.data)
        except Exception as e:
            self._update_cb("???", rsm.Status.ERROR, f"invalid json: {e}")

        if "status" not in contents:
            self._update_cb("???", rsm.Status.ERROR, f"status missing: '{contents}'")

        if "nickname" not in contents:
            self._update_cb("???", rsm.Status.ERROR, f"nickname missing: '{contents}'")

        curr_name = contents["nickname"]
        if self.nickname != curr_name:
            self._update_cb(
                "???",
                rsm.Status.ERROR,
                f"name mismatch: got '{curr_name} vs. '{self.nickname}'",
            )

        status, status_str = rsm.str_to_status(contents["status"])
        note = contents.get("note", "")
        if status_str is not None:
            note += f" ({status_str})"

        self._update_cb(contents.get("node_name", "???"), status, note)


@sc.register_config("json_monitor", name="JsonMonitor", constructor=JsonMonitor)
@dataclass
class JsonMonitorConfig(sc.Config):
    pass
