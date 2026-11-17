import rclpy
from rclpy.node import Node


class HelloPinky(Node):
    def __init__(self):
        super().__init__('hello_pinky')

        self.timer = self.create_timer(
            2.0, self.say_hello
        )

    def say_hello(self):
        self.get_logger().info('Pinky Soccer is running!')


def main(args=None):
    rclpy.init(args=args)

    node = HelloPinky()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
