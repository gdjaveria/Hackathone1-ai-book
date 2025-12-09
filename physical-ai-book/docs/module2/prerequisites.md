---
sidebar_position: 2
---

# Quickstart Guide: Module 2 — The Digital Twin (Gazebo & Unity)

This guide provides a quick overview of the necessary setup to get started with "Module 2 — The Digital Twin (Gazebo & Unity)". To fully engage with the examples and tutorials in this module, you will need to set up a development environment that includes ROS 2, Gazebo, and Unity.

## Prerequisites

Before diving into the module content, ensure you have the following installed and configured on your system:

### 1. ROS 2 Humble/Iron

The module utilizes ROS 2 (Robot Operating System 2) for robotic control and communication.
-   **Installation**: Follow the official ROS 2 installation guide for your operating system (Ubuntu is typically recommended for ROS 2 development).
    -   [ROS 2 Humble Hawksbill Installation Guide](https://docs.ros.org/en/humble/Installation.html)
    -   [ROS 2 Iron Irwini Installation Guide](https://docs.ros.org/en/iron/Installation.html)
-   **Python**: Ensure Python 3.11 (or compatible version with your ROS 2 distribution) is installed and configured correctly. `rclpy` (ROS 2 client library for Python) is used in code examples.

### 2. Gazebo (Ignition Gazebo)

Gazebo is used for physics-based simulation. The module primarily refers to Ignition Gazebo (now Gazebo Sim).
-   **Installation**: Install Gazebo Sim. It typically comes bundled or is easily installable with ROS 2 distributions. If not, refer to the official Gazebo documentation.
    -   [Gazebo Installation Guide](https://gazebosim.org/docs/latest/install)
-   **URDF/SDF**: Familiarity with URDF (Unified Robot Description, Format) or SDF (Simulation Description Format) is beneficial for understanding robot models in Gazebo.

### 3. Unity

Unity will be used for high-fidelity rendering and interactive environments.
-   **Installation**: Download and install Unity Hub, then install a recommended Long Term Support (LTS) version of the Unity Editor.
    -   [Unity Download Page](https://unity3d.com/get-unity/download)
-   **Unity Robotics Packages**: Install relevant Unity Robotics packages (e.g., Unity Robotics ROS-TCP-Connector, URDF-Importer, etc.) through the Unity Package Manager. These packages facilitate integration with ROS 2 and importing robot models.

## Environment Setup Workflow (Conceptual)

1.  **Install ROS 2**: Follow the official guide for your OS.
2.  **Install Gazebo**: Typically installed alongside ROS 2; verify its functionality.
3.  **Install Unity**: Install Unity Hub and the Unity Editor.
4.  **Configure ROS 2 Workspace**: Create a ROS 2 workspace and source your `install/setup.bash` (or `setup.ps1` for Windows).
5.  **Unity Project Setup**: Create a new Unity project and import the necessary Unity Robotics packages.
6.  **Verify Installations**: Run simple test commands or launch basic examples to confirm all components are working correctly.

Throughout the module, specific instructions will be provided for setting up and running individual examples within these environments. This quickstart serves as a high-level guide to ensure your system is prepared.