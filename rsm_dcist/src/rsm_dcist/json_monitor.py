from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional

import ros_system_monitor as rsm
import spark_config as sc
from std_msgs.msg import String


class JsonMonitor:
    """External monitor that listens to json status report."""

    def __init__(
        self,
        config: JsonMonitorConfig,
        nickname: str,
        monitor_name: Optional[str] = None,
    ):
        self.nickname = nickname
        self.monitor_name = monitor_name

    @property
    def relative_name(self):
        if self.monitor_name is None:
            return self.nickname

        return self.nickname.split("/")[1]

    def register_monitor(self, monitor):
        self._update_cb = monitor.update_node_info
        topic = rsm.get_monitor_topic(self.relative_name)
        self.sub = monitor.create_subscription(String, str(topic), self._callback, 1)

    def _callback(self, msg):
        try:
            contents = json.loads(msg.data)
        except Exception as e:
            self._update_cb("???", rsm.Status.ERROR, f"invalid json: {e}")
            return

        if "status" not in contents:
            self._update_cb("???", rsm.Status.ERROR, f"status missing: '{contents}'")
            return

        if "nickname" not in contents:
            self._update_cb("???", rsm.Status.ERROR, f"nickname missing: '{contents}'")
            return

        curr_name = contents["nickname"]
        if self.relative_name != curr_name:
            self._update_cb(
                rsm.TrackedNodeInfo(
                    nickname=self.nickname,
                    last_heartbeat=-1,
                    node_name=contents.get("node_name", "???"),
                    status=rsm.Status.ERROR,
                    notes=f"name mismatch: got '{curr_name} vs. '{self.nickname}'",
                )
            )
            return

        status, status_str = rsm.str_to_status(contents["status"])
        note = contents.get("note", "")
        if status_str is not None:
            note += f" ({status_str})"

        info = rsm.TrackedNodeInfo(
            nickname=self.nickname,
            last_heartbeat=-1,
            node_name=contents.get("node_name", "???"),
            status=status,
            notes=note,
        )
        self._update_cb(info)


@sc.register_config("external_monitor", name="JsonMonitor", constructor=JsonMonitor)
@dataclass
class JsonMonitorConfig(sc.Config):
    pass
