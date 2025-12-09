---
sidebar_position: 4
---

# Chapter 2: Unity Rendering and Interaction

## High-Fidelity Rendering with Unity

Unity is a powerful cross-platform game engine that is widely used for creating 3D simulations, visualizations, and interactive experiences. Its robust rendering pipeline and extensive asset store make it an excellent choice for developing high-fidelity digital twins of humanoid robots, especially when realistic visuals and complex human-robot interaction (HRI) scenarios are required.

In this chapter, we will guide you through setting up a Unity project for robotics, importing a robot model, configuring its rendering properties, and implementing basic interactive controls.

## Unity Installation and Project Setup with Robotics Packages

Assuming you have followed the [prerequisites guide](./prerequisites.md), you should have Unity Hub and a Unity Editor version installed.

### 1. Create a New Unity Project

Open Unity Hub and create a new 3D project. Give it a descriptive name like "RobotDigitalTwin".

<!-- TODO: Add screenshot of Unity Hub with new project creation -->

### 2. Install Unity Robotics Packages

Unity provides several packages that facilitate robotics development and integration with ROS 2.
-   **Open Package Manager**: In your Unity project, go to `Window > Package Manager`.
-   **Add Package by Name**: Click the `+` icon in the top-left corner and select "Add package by name...".
-   **Install Essential Packages**:
    -   `com.unity.robotics.ros-tcp-connector`: Enables communication between Unity and ROS 2.
    -   `com.unity.robotics.urdf-importer`: Allows importing URDF files directly into Unity.
    -   `com.unity.robotics.visualizations`: Provides tools for visualizing ROS data.

<!-- TODO: Add screenshot of Unity Package Manager with robotics packages installed -->

## Importing a Robot Model and Configuring Rendering Properties

Once the URDF Importer package is installed, you can import your robot model.

### 1. Import URDF Model

-   In your Unity project, go to `Robotics > URDF Importer > Import URDF`.
-   Select your `simple_robot.urdf` file (created in Chapter 1) from `physical-ai-book/docs/module2/code/`.
-   Configure import settings (e.g., collision properties, physics settings) and click "Import".

<!-- TODO: Add screenshot of imported robot model in Unity scene -->

### 2. Configure Rendering

After importing, your robot model will appear in the Project window. Drag it into your scene.
-   **Materials and Shaders**: Adjust materials and shaders to achieve realistic visual appearance. Unity's High-Definition Render Pipeline (HDRP) or Universal Render Pipeline (URP) can be used for advanced rendering.
-   **Lighting**: Set up realistic lighting in your scene to enhance the visual fidelity of your robot.
-   **Cameras**: Position and configure cameras to capture the desired views of your robot and environment.

<!-- TODO: Add screenshot of rendered robot model with configured lighting and materials -->

## Implementing Basic Interactive Controls

To interact with your robot model, you can create simple Unity scripts.

### 1. Create a C# Script

-   In the Project window, right-click and select `Create > C# Script`. Name it `RobotController.cs`.
-   Attach this script to your robot's root GameObject in the scene.

<!-- TODO: Add screenshot of RobotController script attached to robot GameObject -->

### 2. Example: Simple Movement Script

Here's a basic C# script that allows you to move and rotate the robot using keyboard inputs.

```csharp
using UnityEngine;

public class RobotController : MonoBehaviour
{
    public float moveSpeed = 5.0f;
    public float rotateSpeed = 100.0f;

    // Update is called once per frame
    void Update()
    {
        // Translate the robot
        float horizontal = Input.GetAxis("Horizontal"); // A/D or Left/Right Arrow keys
        float vertical = Input.GetAxis("Vertical");     // W/S or Up/Down Arrow keys

        Vector3 moveDirection = new Vector3(horizontal, 0, vertical);
        transform.Translate(moveDirection * moveSpeed * Time.deltaTime);

        // Rotate the robot (optional, if you want rotation)
        // float rotationInput = Input.GetAxis("Mouse X"); // Example with mouse input
        // transform.Rotate(Vector3.up, rotationInput * rotateSpeed * Time.deltaTime);
    }
}
```
<!-- TODO: Add screenshot of Unity scene with robot being controlled interactively -->