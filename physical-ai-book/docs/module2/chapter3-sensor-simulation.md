---
sidebar_position: 5
---

# Chapter 3: Sensor Simulation Workflows

## Introduction to Sensor Simulation

Sensors are the eyes and ears of a robot, providing crucial information about its environment and internal state. In digital twin simulations, accurately simulating sensor data is vital for developing robust perception and control algorithms. This allows us to test how a robot would interpret its surroundings without the need for physical hardware, making development faster, safer, and more cost-effective.

This chapter will introduce you to simulating common robotics sensors: LiDAR (Light Detection and Ranging), Depth Cameras, and IMUs (Inertial Measurement Units), using both Gazebo and Unity. You will learn how to integrate these virtual sensors into your robot models and interpret the data they provide.

## Simulating LiDAR Sensors

LiDAR sensors measure distances to objects by emitting pulsed laser light and measuring the time it takes for the reflected light to return. This creates a 3D point cloud of the environment.

### Integration in Gazebo

Gazebo provides a `ray` sensor type that can be configured to simulate LiDAR.

```xml
<!-- Example: Add a simple LiDAR sensor to your URDF -->
<link name="lidar_link">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.5707</min_angle>
          <max_angle>1.5707</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_laser.so">
      <topicName>/scan</topicName>
      <frameName>lidar_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```
<!-- TODO: Add screenshot of LiDAR point cloud visualization in Gazebo -->

### Integration in Unity

Unity can simulate LiDAR using raycasting. Robotics packages often provide components for this.
<!-- TODO: Add details on Unity LiDAR simulation using raycasting or specific packages -->

## Simulating Depth Cameras

Depth cameras provide an image where each pixel's value represents the distance from the camera to the object.

### Integration in Gazebo

Gazebo supports depth cameras through the `camera` sensor type with specific plugins.
<!-- TODO: Add details on Gazebo Depth Camera configuration and relevant plugins -->

### Integration in Unity

Unity can simulate depth cameras by rendering depth textures.
<!-- TODO: Add details on Unity Depth Camera simulation -->

## Simulating IMU Sensors

An IMU (Inertial Measurement Unit) measures a robot's orientation, angular velocity, and linear acceleration.

### Integration in Gazebo

Gazebo has a dedicated `imu` sensor type.
<!-- TODO: Add details on Gazebo IMU sensor configuration -->

### Integration in Unity

Unity provides access to device's IMU data (if running on a device) or can be simulated through physics.
<!-- TODO: Add details on Unity IMU simulation or data access -->

## Using `sensor_example.py` to Visualize Data

You can use the `sensor_example.py` script to subscribe to and visualize data from simulated LiDAR, Depth, and IMU sensors in a ROS 2 environment.

To run this script, first ensure you have a Gazebo or Unity simulation running that publishes sensor data to the respective ROS 2 topics (`/scan`, `/camera/depth/image_raw`, `/imu/data`).

Save the following code as `sensor_example.py` in your `physical-ai-book/docs/module2/code/` directory.

```python
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
```

To run the visualizer:
1.  Ensure ROS 2 environment is sourced.
2.  Launch your Gazebo or Unity simulation with sensor publishing.
3.  In a new terminal, navigate to `physical-ai-book/docs/module2/code/`.
4.  Run: `python3 sensor_example.py`

<!-- TODO: Add screenshot of sensor_example.py output for LiDAR (terminal output) -->
<!-- TODO: Add screenshot of sensor_example.py output for Depth Camera (OpenCV window) -->
<!-- TODO: Add screenshot of sensor_example.py output for IMU (terminal output) -->
