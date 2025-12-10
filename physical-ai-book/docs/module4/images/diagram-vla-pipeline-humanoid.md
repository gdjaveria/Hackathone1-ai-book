# Diagram: Overall VLA Pipeline for Autonomous Humanoid

## Description

This diagram illustrates the complete Vision-Language-Action (VLA) pipeline for an autonomous humanoid robot, from human voice command to physical execution and feedback.

## Components

1.  **Human Command (Voice)**: The initial input from the user.
2.  **Voice-to-Text (STT)**: Processes audio to text (e.g., Whisper).
3.  **Natural Language Understanding (NLU)**: Extracts intent and parameters from text.
4.  **Actionable Intent**: Structured representation of the command.
5.  **LLM Cognitive Planning**: Translates actionable intent into an abstract plan (Task Decomposition & Chain-of-Thought).
6.  **Abstract Plan (Primitives)**: Sequence of high-level, robot-agnostic actions.
7.  **Custom Planners / ROS 2 Action Mapping**: Converts abstract primitives to specific ROS 2 actions (e.g., Nav2 goals, MoveIt 2 actions).
8.  **Robot Execution (ROS 2)**: The humanoid robot's subsystems execute ROS 2 commands.
9.  **Perception & Feedback**: Robot sensors gather data, and execution status is monitored.
10. **Physical Action**: Robot interacts with the environment.

## Flow

A human issues a voice command. This command is converted to text by the STT module, and then to a structured actionable intent by the NLU module. The LLM's cognitive planning then processes this intent, breaking it down into an abstract plan. Custom planners map these abstract primitives to ROS 2 actions, which the robot executes. Throughout execution, perception and feedback mechanisms constantly monitor the environment and robot state, feeding information back to the planning system and potentially to the LLM for adaptive re-planning. The cycle culminates in the robot performing physical actions in its environment.

## Visual Representation (Conceptual)

```mermaid
graph LR
    A[Human Command (Voice)] --> B{Voice-to-Text (STT)};
    B --> C{Natural Language Understanding (NLU)};
    C --> D[Actionable Intent];
    D --> E{LLM Cognitive Planning};
    E --> F[Abstract Plan (Primitives)];
    F --> G{Custom Planners / ROS 2 Action Mapping};
    G --> H[Robot Execution (ROS 2)];
    H --> I{Perception & Feedback};
    I --> E;
    H --> J[Physical Action];
```
