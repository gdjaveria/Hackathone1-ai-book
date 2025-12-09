import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorDataVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_data_visualizer')
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Sensor Data Visualizer node started.')

    def lidar_callback(self, msg: LaserScan):
        # Process and visualize LiDAR data (e.g., print min/max range, simple plot)
        if msg.ranges:
            min_range = min(msg.ranges)
            max_range = max(msg.ranges)
            self.get_logger().info(f'LiDAR: Min Range: {min_range:.2f}m, Max Range: {max_range:.2f}m')
            # For actual visualization, you'd use a plotting library or Rviz
        
    def depth_callback(self, msg: Image):
        # Convert ROS Image message to OpenCV image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Normalize for display (if float depth image)
            depth_display = cv2.normalize(depth_image, None, 255,0, cv2.NORM_MINMAX, cv2.CV_8U)
            depth_display = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
            cv2.imshow("Depth Image", depth_display)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Could not convert depth image: {e}')

    def imu_callback(self, msg: Imu):
        # Process and visualize IMU data (e.g., print orientation, acceleration)
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        self.get_logger().info(
            f'IMU: Orientation (x,y,z,w): ({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f}, {orientation.w:.2f}), '
            f'Angular Velocity (x,y,z): ({angular_velocity.x:.2f}, {angular_velocity.y:.2f}, {angular_velocity.z:.2f}), '
            f'Linear Acceleration (x,y,z): ({linear_acceleration.x:.2f}, {linear_acceleration.y:.2f}, {linear_acceleration.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    sensor_visualizer = SensorDataVisualizer()
    rclpy.spin(sensor_visualizer)
    sensor_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
