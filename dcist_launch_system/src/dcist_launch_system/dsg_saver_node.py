#!/usr/bin/env python3
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy
import threading
from datetime import datetime
from dcist_launch_system_msgs.srv import SaveDsg, SaveDsg_Response

from hydra_ros import DsgSubscriber
from rclpy.node import Node
import os
from ros_system_monitor_msgs.msg import NodeInfoMsg


class DsgSaver(Node):
    def __init__(self):
        super().__init__("dsg_saver")

        self.n_saves = 0

        # Declare and get parameters
        self.declare_parameter("dsg_save_dir", "")
        self.dsg_save_dir = self.get_parameter("dsg_save_dir").get_parameter_value().string_value
        assert self.dsg_save_dir != "", "You need to pass dsg_save_dir path"

        if not os.path.exists(self.dsg_save_dir):
            self.get_logger().warning(
                f"Save directory does not exist! Making {self.dsg_save_dir}"
            )
            os.makedirs(self.dsg_save_dir)

        self.heartbeat_pub = self.create_publisher(NodeInfoMsg, "~/node_status", 1)
        heartbeat_timer_group = MutuallyExclusiveCallbackGroup()
        timer_period_s = 0.1
        self.timer = self.create_timer(
            timer_period_s, self.hb_callback, callback_group=heartbeat_timer_group
        )

        self.dsg = None
        self.dsg_lock = threading.Lock()
        self.last_dsg_time = None
        DsgSubscriber(self, "~/dsg_in", self.dsg_callback)

        self.dsg_save_server = self.create_service(
            SaveDsg,
            '~/save_dsg',
            self.save_cb
        )

    def dsg_callback(self, header, dsg):
        with self.dsg_lock:
            self.dsg = dsg
            self.last_dsg_time = self.get_clock().now().nanoseconds * 1e-9

    def save_cb(self, req, rep: SaveDsg_Response):
        self.get_logger().info("Got request to save dsg")

        with self.dsg_lock:
            if self.dsg is None:
                self.get_logger().error("Cannot save, no received DSG!")
                rep.success = False
                return rep
            t_now = self.get_clock().now().nanoseconds * 1e-9
            dt = t_now - self.last_dsg_time
            if dt > 10:
                self.get_logger().warning(f"Saving old DSG ({dt} s old)")
    
            if req.save_path == "":
                now = datetime.now()
                fn = now.strftime("%Y-%m-%d-%H_%M_%S_dsg.json")
                full_filepath = os.path.join(self.dsg_save_dir, fn)
            else:
                full_filepath = req.save_path

            self.get_logger().info("Saving dsg to %s" % full_filepath)
            self.dsg.save(full_filepath, req.include_mesh)
            self.n_saves += 1
            rep.success = True
            self.get_logger().info("Finished saving DSG")
            return rep


    def hb_callback(self):

        if self.dsg is None:
            status = NodeInfoMsg.WARNING
            notes = "No confirmed DSG connection..."
        else:
            t_now = self.get_clock().now().nanoseconds * 1e-9
            dt = t_now - self.last_dsg_time
            if dt > 10:
                status = NodeInfoMsg.WARNING
                notes = "Have old DSG ({dt} s). {self.n_saves} saved."
            else:
                status = NodeInfoMsg.NOMINAL
                notes = f"Confirmed DSG connection. {self.n_saves} saved."

        msg = NodeInfoMsg()
        msg.nickname = "dsg_saver"
        msg.node_name = self.get_fully_qualified_name()
        msg.status = status
        msg.notes = notes
        self.heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DsgSaver()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        finally:

            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
