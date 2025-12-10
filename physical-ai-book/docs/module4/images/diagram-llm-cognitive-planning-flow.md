# Diagram: LLM-driven Cognitive Planning Flow

## Description

This diagram illustrates how an LLM performs cognitive planning to translate high-level commands into ROS 2 actions for a robot.

## Components

1.  **Actionable Intent Input**: Structured intent received from the Voice-to-Action stage (e.g., `Intent: Navigate`, `Destination: kitchen`).
2.  **LLM (Large Language Model)**: The core AI model responsible for cognitive reasoning.
    *   **Task Decomposition**: Breaks down complex commands into simpler sub-tasks.
    *   **Chain-of-Thought (CoT)**: Provides step-by-step reasoning for the decomposition.
3.  **Abstract Plan Generation**: A sequence of high-level, robot-agnostic primitives (e.g., `move_to(location)`, `grasp_object(object_id)`).
4.  **Custom Planners / ROS 2 Mapping**: Modules that translate abstract primitives into specific ROS 2 `Action` goals or `Service` requests.
5.  **Robot Execution**: The robot's hardware and software stack performing the low-level ROS 2 commands.
6.  **Feedback Loop**: Robot execution status and sensor data are fed back to the planning system, potentially to the LLM for adjustments.

## Flow

The process begins with a structured, actionable intent. This intent is fed into the LLM, which uses task decomposition and chain-of-thought reasoning to generate a sequence of abstract primitives. These primitives form an abstract plan. Each primitive is then handed to a custom planner (or executor) that maps it to concrete ROS 2 actions or service calls. The robot executes these ROS 2 commands, and the execution status and environmental feedback are sent back to the planning system, creating a continuous feedback loop.

## Visual Representation (Conceptual)

```mermaid
graph TD
    A[Actionable Intent Input] --> B{LLM (Task Decomposition & CoT)};
    B --> C[Abstract Plan (Primitives)];
    C --> D{Custom Planners / ROS 2 Mapping};
    D --> E[Robot Execution (ROS 2 Actions/Services)];
    E --> F[Feedback Loop (Status/Sensor Data)];
    F --> B;
```
