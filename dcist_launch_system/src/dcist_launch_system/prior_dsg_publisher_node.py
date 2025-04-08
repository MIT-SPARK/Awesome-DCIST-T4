#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import spark_dsg
#from autoabstr.utils.pipeline import Impl
from hydra_ros import DsgPublisher

class PriorDsgPublisher(Node):
    def __init__(self):
        super().__init__('prior_dsg_publisher')
        
        # Declare and get parameters
        self.declare_parameter('prior_dsg_fn', "")
        self.declare_parameter('run_autoabstr', False)
        self.declare_parameter('autoabstr_config_filepath', "")

        self.prior_dsg_fn = self.get_parameter('prior_dsg_fn').get_parameter_value().string_value

        # Load the DSG from file
        self.G = spark_dsg.DynamicSceneGraph.load(self.prior_dsg_fn)

        run_autoabstr = self.get_parameter('run_autoabstr').get_parameter_value().bool_value
        if run_autoabstr:
            self.config_filepath = self.get_parameter('autoabstr_config_filepath').get_parameter_value().string_value
            # Create the auto-abstraction labeler
            self.autoabstr_labeler = Impl(self.config_filepath)
        
            # Add the automatic abstractions
            success = self.autoabstr_labeler.step(self.G)
            if success:
                self.autoabstr_labeler.step_adj()

            self.G = self.autoabstr_labeler.graph
        
        self.dsg_sender = DsgPublisher(self, "~/dsg_out", True)

        # Timer to publish DSG at regular intervals
        self.timer = self.create_timer(3.0, self.publish_dsg)
        
    def publish_dsg(self):
        self.dsg_sender.publish(self.G)
        self.get_logger().info('Published map')


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = PriorDsgPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up before shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
