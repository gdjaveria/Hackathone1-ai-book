# Quickstart Guide: The AI-Robot Brain (NVIDIA Isaac) Module

**Feature Branch**: `001-isaac-robot-brain`  
**Date**: December 10, 2025
**Purpose**: To provide a high-level guide for readers to quickly get started with the concepts and code examples presented in "The AI-Robot Brain (NVIDIA Isaac)" module.

## 1. Accessing the Module Content

This module will be integrated into the main `physical-ai-book` Docusaurus project.

-   **Online Access**: The most straightforward way to access the completed module will be through the published `physical-ai-book` website. Specific links to the chapters will be provided once available.
-   **Local Development**: For those wishing to view the module locally or contribute:
    1.  Clone the `hackathone` repository (if not already done).
    2.  Navigate to the `physical-ai-book` directory.
    3.  Follow the Docusaurus setup instructions in the `physical-ai-book/README.md` (e.g., `npm install`, `npm start`).
    4.  The module's chapters will appear under the relevant section in the local Docusaurus build.

## 2. Running Code Examples

The module will include Python-based code examples primarily using ROS 2, NVIDIA Isaac Sim, Isaac ROS, and Nav2.

-   **Environment Setup**:
    -   Ensure you have a compatible ROS 2 (Humble/Iron) environment set up.
    -   NVIDIA Isaac Sim and Isaac ROS typically require specific NVIDIA hardware and driver setups. Refer to the official NVIDIA Isaac documentation for detailed installation instructions.
    -   Nav2 is a ROS 2 package; installation generally follows standard ROS 2 package installation procedures.
-   **Example Locations**: Code examples will be provided within the module's chapters (e.g., in `physical-ai-book/docs/moduleX/code/`).
-   **Execution**: Each code example will include specific instructions for setup and execution within its respective chapter. This may involve:
    -   Activating a ROS 2 workspace.
    -   Running Isaac Sim with specific scenarios.
    -   Launching ROS 2 nodes (e.g., `ros2 run ...`, `ros2 launch ...`).

## 3. Key Concepts to Understand

Before diving deep into the code, it is recommended to familiarize yourself with the foundational concepts:

-   **ROS 2 Basics**: Nodes, topics, services, actions.
-   **Robotics Simulation**: Importance and benefits of using simulators like Isaac Sim.
-   **SLAM**: Simultaneous Localization and Mapping, especially VSLAM.
-   **Robot Navigation**: Path planning, localization, obstacle avoidance principles.

Further details on these and other prerequisites will be covered within the module's introductory sections.
