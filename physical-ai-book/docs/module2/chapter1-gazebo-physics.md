---
sidebar_position: 3
---

# Chapter 1: Gazebo Physics and Environment Setup

## Introduction to Gazebo for Robotics Simulation

Gazebo is a powerful 3D robotics simulator that allows you to accurately simulate complex robot systems in realistic environments. It offers a robust physics engine, high-quality graphics, and convenient interfaces for users and programs. For humanoid robotics, Gazebo provides a crucial platform for testing algorithms, prototyping designs, and generating data in a safe and reproducible manner.

In this chapter, we will guide you through setting up a basic Gazebo environment, creating a simple robot model, and configuring its physical properties to observe realistic interactions within the simulation.

## Basic Gazebo Installation and Environment Verification

Assuming you have followed the [prerequisites guide](./prerequisites.md), Gazebo (Ignition Gazebo / Gazebo Sim) should be installed alongside your ROS 2 distribution.

To verify your Gazebo installation, open a terminal and run:

```bash
gazebo
```

This command should launch the Gazebo GUI, presenting you with an empty world. If Gazebo launches successfully, your basic installation is working.

<!-- TODO: Add screenshot of Gazebo GUI with an empty world -->

## Creating a Simple Robot Model with URDF

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. It specifies the robot's kinematics, dynamics, visual appearance, and collision properties.

Let's create a simple URDF for a basic block-like robot. Save this content as `simple_robot.urdf` in your `physical-ai-book/docs/module2/code/` directory.

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1.0" />
      <inertia ixx="0.00666666666667" ixy="0" ixz="0" iyy="0.00666666666667" iyz="0" izz="0.00666666666667" />
    </inertial>
  </link>
</robot>
```
<!-- TODO: Add screenshot of the simple robot in Gazebo -->

To view this robot in Gazebo, you can use `ros2 launch` with a minimal launch file. Create `view_robot.launch.py` in your `physical-ai-book/docs/module2/code/` directory:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('your_package_name') # Replace 'your_package_name' with your ROS 2 package name
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf') # Assuming urdf in a 'urdf' subfolder

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}],
            arguments=[urdf_file]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'simple_robot', '-file', urdf_file]
        )
    ])
```
*(Note: You'll need to create a simple ROS 2 package and place these files in appropriate subdirectories like `urdf` and `launch`. This example is conceptual for demonstration.)*

## Configuring Gravity and Collision Properties

Gazebo's physics engine handles interactions based on the collision shapes and inertial properties defined in the URDF, as well as the world's gravity settings.

The URDF above defines a `collision` element, which is critical for physical interactions. The `inertial` element defines the mass and inertia tensor, influencing how the object reacts to forces.

**Gravity**: By default, Gazebo worlds have gravity enabled. You can observe its effect by simply spawning your robot model. If it's not supported by anything, it will fall.

**Collisions**: When two `collision` geometries intersect, the physics engine calculates contact forces to prevent interpenetration. The `box` geometry in our URDF provides a simple collision shape.

*(Further examples will show how to create more complex environments and interaction scenarios, as well as customize physics parameters in Gazebo world files.)*