import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

class Example1Node(Node):
    def __init__(self):
        super().__init__('example1_node')
        self.publisher = self.create_publisher(String, '~/topic', 1)

        param_descriptor = ParameterDescriptor(
            type=Parameter.Type.STRING.value, 
            description="This is a string",
        )
        self.declare_parameter('prefix', descriptor=param_descriptor)

        self.prefix = self.get_parameter('prefix').value
        timer_period_s = 0.5
        self.timer = self.create_timer(timer_period_s, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = self.prefix + 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = Example1Node()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

