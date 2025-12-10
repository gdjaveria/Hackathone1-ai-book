# Diagram: Voice-to-Action Flow

## Description

This diagram illustrates the process of converting spoken commands into actionable intent for a robot.

## Components

1.  **Speech Input**: User's spoken command.
2.  **Speech-to-Text (STT)**: A component (e.g., Whisper) that converts audio waves into textual transcription.
3.  **Natural Language Understanding (NLU)**: A component that processes the text to identify the user's intent and extract relevant entities (slots).
    *   **Intent Classification**: Determines the overall goal (e.g., `Navigate`, `Manipulate`).
    *   **Slot Filling**: Extracts parameters (e.g., `Destination: kitchen`, `Object: red cube`).
4.  **Actionable Intent**: A structured representation of the command (e.g., JSON or a similar data structure) that can be understood by the robot's planning system.

## Flow

The flow starts with the user providing a spoken command. This audio is captured and fed into the STT system (Whisper), which transcribes it into text. The transcribed text then goes to the NLU system, where intent classification identifies the primary goal and slot filling extracts key details. Finally, a structured, actionable intent is generated and passed to the next stage of the VLA pipeline.

## Visual Representation (Conceptual)

```mermaid
graph TD
    A[Speech Input] --> B{Speech-to-Text (Whisper)};
    B --> C{Natural Language Understanding};
    C --> D{Intent Classification};
    C --> E{Slot Filling};
    D --> F[Actionable Intent];
    E --> F;
```
