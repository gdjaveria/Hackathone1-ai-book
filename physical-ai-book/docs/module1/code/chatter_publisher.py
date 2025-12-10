import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = Node('chatter_publisher')
    publisher = node.create_publisher(String, 'chatter', 10)
    msg = String()
    msg.data = 'Hello from rclpy script!'
    publisher.publish(msg)
    node.get_logger().info('Published message: "%s"' % msg.data)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
