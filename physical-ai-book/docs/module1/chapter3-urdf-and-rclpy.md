---
sidebar_position: 4
---

# Chapter 3: URDF and rclpy

In this chapter, we will cover two important topics: Unified Robot Description Format (URDF) for modeling our robot, and `rclpy` for bridging Python AI with ROS 2.

## URDF (Unified Robot Description Format)

**URDF** is an XML format used in ROS to describe the physical structure of a robot. It defines the robot's links (the rigid parts) and joints (the connections between links), as well as their properties like size, shape, color, and mass.

### Simple URDF Example

Here is a simple example of a two-link robot described in URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="link_1">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_link1" type="fixed">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.3"/>
  </joint>

</robot>
```

This URDF file defines a `base_link` (a cylinder) and `link_1` (a box), connected by a fixed joint.

### Viewing a URDF in RViz2

To view a URDF file, you can use RViz2, the 3D visualizer for ROS 2. You will need to create a launch file to start RViz2 with your robot description.

Here is an example of a launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    robot_description = ParameterValue(Command(['xacro ', 'my_robot.urdf.xacro']), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'my_robot.rviz']
        ),
    ])
```

## rclpy: Python interface for ROS 2

We have been using `rclpy` throughout these examples. It is the official Python client library for ROS 2. It allows you to write ROS 2 nodes, publishers, subscribers, services, and clients in Python, making it easy to integrate your Python-based AI code with your ROS 2 system.

We will continue to use `rclpy` in the upcoming modules to build the brain of our humanoid robot.