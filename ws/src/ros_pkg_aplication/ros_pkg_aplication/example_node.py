import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')

        # Declare the 'my_param1' parameter
        self.declare_parameter('my_param1', 0.0)

        # Get the 'my_param1' parameter
        my_param1 = self.get_parameter('my_param1').get_parameter_value()

        # Print the parameter value
        self.get_logger().info('my_param1: {}'.format(my_param1.double_value))

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()