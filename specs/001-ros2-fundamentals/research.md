# Research and Decisions for Module 1

This document records the high-level architectural and content decisions for Module 1 of the "Physical AI & Humanoid Robotics Book".

## Docusaurus Structure

- **Decision**: The book will be built using Docusaurus.
- **Rationale**: Docusaurus is a popular, easy-to-use static site generator that is well-suited for documentation and books. It provides features like versioning, search, and a good user experience out of the box.
- **Sidebar Layout**: The sidebar will be organized by modules, with each module containing its chapters. This will provide a clear and hierarchical navigation structure for the reader.

## Code Example Format

- **Decision**: All code examples will follow ROS 2 (Humble/Iron) conventions. `rclpy` will be used for Python-based ROS 2 nodes.
- **Rationale**: Sticking to official conventions ensures that the code is idiomatic and will be familiar to those already working with ROS 2. `rclpy` is the official Python client library for ROS 2.

## Diagram Style

- **Decision**: Diagrams will be created using Mermaid.js where possible. For more complex diagrams, high-quality images will be used.
- **Rationale**: Mermaid.js allows diagrams to be created and versioned as code, which is ideal for a project managed with Git. It is also well-supported by Docusaurus.

## Versioning Strategy

- **Decision**: The book will be versioned using semantic versioning (MAJOR.MINOR.PATCH).
- **Rationale**: Semantic versioning provides a clear way to communicate the scope of changes between different versions of the book.

## Workflow

- **Decision**: The content will be developed using an iterative "write-as-you-build" workflow with Spec-Kit Plus. Chapters will be generated via an AI agent (like Gemini or Claude) and then refined through the spec-driven process.
- **Rationale**: This AI-native workflow will accelerate content creation while ensuring high quality through the structured refinement process.
