import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class Example2Node(Node):
    def __init__(self):
        super().__init__('example2_node')
        self.subscriber = self.create_subscription(String, '~/topic_in', self.callback, 1)

    def callback(self, msg):
        print(msg.data) 

def main(args=None):
    rclpy.init(args=args)
    node = Example2Node()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

