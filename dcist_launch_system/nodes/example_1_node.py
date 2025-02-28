import rclpy
from dcist_launch_system.example_1 import Example1Node


def main(args=None):

    rclpy.init(args=args)
    node = Example1Node()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
