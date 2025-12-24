# Tasks: Module 1 — The Robotic Nervous System (ROS 2)

**Input**: Design documents from `specs/001-ros2-fundamentals/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Setup (Shared Infrastructure)

- [X] T001 Initialize Docusaurus project in the repository root.
- [X] T002 Configure Docusaurus sidebar for Module 1 in `docusaurus.config.js`.
- [X] T003 Create the directory structure for Module 1 content in `docs/module1/`.

---

## Phase 2: Foundational (Blocking Prerequisites)

- [X] T004 Write the introduction to Module 1 in `docs/module1/introduction.md`.
- [X] T005 Create a placeholder file for each chapter in `docs/module1/`.

---

## Phase 3: User Story 1 - Understand ROS 2 Nodes (Priority: P1) 🎯 MVP

**Goal**: Create a chapter explaining ROS 2 Nodes.

**Independent Test**: The user can write a simple "hello world" ROS 2 node and run it.

### Implementation for User Story 1

- [X] T006 [US1] Draft the content for the ROS 2 Nodes chapter in `docs/module1/chapter1-ros2-nodes.md`.
- [X] T007 [US1] Create a runnable "hello world" code example for a ROS 2 Node.
- [X] T008 [US1] Add a diagram illustrating the structure of a ROS 2 Node.

---

## Phase 4: User Story 2 - Communicate with Topics and Services (Priority: P1)

**Goal**: Create a chapter explaining ROS 2 Topics and Services.

**Independent Test**: The user can create two nodes that communicate with each other using both a topic and a service.

### Implementation for User Story 2

- [X] T009 [US2] Draft the content for the ROS 2 Topics and Services chapter in `docs/module1/chapter2-ros2-topics-services.md`.
- [X] T010 [US2] Create a runnable code example for a publisher and subscriber.
- [X] T011 [US2] Create a runnable code example for a service server and client.
- [X] T012 [US2] Add diagrams illustrating the publisher/subscriber and service/client communication models.

---

## Phase 5: User Story 3 - Model a Robot with URDF (Priority: P2)

**Goal**: Create a chapter explaining URDF.

**Independent Test**: The user can create a simple URDF file for a multi-link robot and view it in RViz2.

### Implementation for User Story 3

- [X] T013 [US3] Draft the content for the URDF chapter in `docs/module1/chapter3-urdf-and-rclpy.md`.
- [X] T014 [US3] Create a simple URDF file for a humanoid component.
- [X] T015 [US3] Provide instructions on how to view the URDF in RViz2.

---

## Phase 6: User Story 4 - Bridge AI to ROS with rclpy (Priority: P2)

**Goal**: Create content explaining how to bridge AI to ROS with rclpy.

**Independent Test**: The user can create a Python script that uses `rclpy` to send a command to a ROS 2 node.

### Implementation for User Story 4

- [X] T016 [US4] Add a section to `docs/module1/chapter3-urdf-and-rclpy.md` explaining `rclpy`.
- [X] T017 [US4] Create a runnable code example of a Python script using `rclpy` to communicate with a ROS 2 node.

---

## Phase N: Polish & Cross-Cutting Concerns

- [X] T018 Review all chapters for clarity, consistency, and technical accuracy.
- [X] T019 Validate all code examples in a ROS 2 workspace.
- [X] T020 Check for Docusaurus build errors.
- [X] T021 Ensure all content aligns with the project constitution.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must be completed before any other phase.
- **Phase 2 (Foundational)** must be completed before the user story phases.
- **User Story Phases (3-6)** can be worked on in parallel after Phase 2 is complete.
- **Phase N (Polish)** should be done after all other phases are complete.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently.

### Incremental Delivery

1.  Complete Setup + Foundational.
2.  Add User Story 1 & 2 -> Test Independently -> Demo
3.  Add User Story 3 & 4 -> Test Independently -> Demo
