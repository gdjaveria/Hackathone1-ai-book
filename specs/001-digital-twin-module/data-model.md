# Data Model: Module 2 — The Digital Twin (Gazebo & Unity)

This document describes the conceptual data model for the content structure of Module 2, which focuses on Digital Twins using Gazebo and Unity. Given that this feature is a documentation module, the "data model" represents the organization and relationships between the educational content components rather than a traditional software database schema.

## Entities

### Entity: Module 2 - The Digital Twin

-   **Description**: The top-level conceptual container for all educational content related to Digital Twins, specifically covering Gazebo physics simulation, Unity high-fidelity interaction, and various sensor simulations.
-   **Attributes**:
    -   `Title` (String): "Module 2 — The Digital Twin (Gazebo & Unity)"
    -   `TargetAudience` (String): "Beginners–intermediate learners building humanoid robot simulations and virtual environments."
    -   `FocusAreas` (List of Strings): Key technical areas covered, including "Gazebo physics simulation", "Unity high-fidelity interaction", and "Sensor simulation (LiDAR, Depth, IMU)".
    -   `ChapterCount` (Integer): The number of distinct chapters within this module (expected to be 2-3).
-   **Relationships**:
    -   **Contains**: One-to-many relationship with `Chapter` entities.

### Entity: Chapter

-   **Description**: A major logical division within the `Module 2` content, focusing on a specific thematic area (e.g., Gazebo setup, Unity rendering, Sensor simulation). Each chapter is designed to be a self-contained learning unit.
-   **Attributes**:
    -   `Title` (String): A descriptive title for the chapter (e.g., "Chapter 1: Gazebo Physics and Environment Setup").
    -   `Order` (Integer): The sequential position of the chapter within the module.
    -   `FilePath` (String): The relative path to the main Markdown file for the chapter (e.g., `chapter1-gazebo-physics.md`).
-   **Relationships**:
    -   **Belongs to**: One-to-one relationship with `Module 2 - The Digital Twin`.
    -   **Contains**: One-to-many relationship with `Section` entities.
    -   **References**: Many-to-many relationship with `Code Example` entities.

### Entity: Section

-   **Description**: A sub-division within a `Chapter`, detailing a specific concept, sub-topic, or step-by-step instruction set. Sections help break down complex topics into digestible parts.
-   **Attributes**:
    -   `Title` (String): A descriptive title for the section (e.g., "2.1 Configuring Robot Physics").
    -   `Order` (String/Integer): A hierarchical identifier or sequential position within its parent chapter.
-   **Relationships**:
    -   **Belongs to**: One-to-one relationship with `Chapter`.
    -   **May contain**: Many-to-many relationship with `Code Example` entities.

### Entity: Code Example

-   **Description**: Illustrative code snippets or complete program files that demonstrate technical concepts, configurations, or workflows discussed in `Section`s or `Chapter`s.
-   **Attributes**:
    -   `Name` (String): A descriptive name for the example (e.g., "Simple Gazebo Robot URDF").
    -   `FilePath` (String): The relative path to the code file within the module's `code/` directory (e.g., `code/gazebo_example.py`).
    -   `Language` (String): The programming language of the example (e.g., "Python", "C#").
    -   `Purpose` (String): A brief explanation of what the code example demonstrates or achieves.
-   **Relationships**:
    -   **Associated with**: Many-to-many relationship with `Section` (and implicitly `Chapter`).
