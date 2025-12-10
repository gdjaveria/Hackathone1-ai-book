from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
