# Research Findings: Module 2 — The Digital Twin (Gazebo & Unity)

## Docusaurus Site Responsiveness Metrics

**Decision**: We will target the following quantitative metrics for Docusaurus site responsiveness to ensure a smooth user experience for beginner-intermediate learners. These metrics are aligned with Core Web Vitals and general web performance best practices.

**Rationale**: These metrics directly correlate with user perception of site speed and interactivity. By aiming for these targets, we ensure the learning material is accessible and not hindered by poor web performance.

**Metrics Chosen**:
-   **First Contentful Paint (FCP)**: Under 1.8 seconds.
-   **Largest Contentful Paint (LCP)**: Under 2.5 seconds.
-   **Time to Interactive (TTI)**: Under 3.8 seconds.
-   **Cumulative Layout Shift (CLS)**: Below 0.1.
-   **Total Blocking Time (TBT)**: Under 200 milliseconds.
-   **Page Load Time (Full Load)**: Under 5 seconds.

**Alternatives Considered**:
-   *Strictly theoretical benchmarks*: Rejected, as practical user experience metrics are more relevant for a documentation site.
-   *Ignoring performance metrics*: Rejected, as poor performance can significantly detract from the learning experience.

## Simulation Example Performance Metrics

**Decision**: We will establish the following performance guidelines for the simulation examples to ensure they are runnable and provide a good learning experience on typical beginner-intermediate learner hardware.

**Rationale**: The target audience may not have high-end machines. Setting realistic performance expectations ensures that learners can execute the examples without undue frustration due to long runtimes or resource exhaustion.

**Metrics Chosen**:
-   **Execution Time (Wall-Clock Time)**: Examples should complete within reasonable timeframes (e.g., under 30 seconds for simple scenarios on typical laptops).
-   **CPU Usage (Average Percentage)**: Examples should run without consistently pegging CPU at 100% on a single core; aim for moderate usage allowing for system responsiveness.
-   **Memory Usage (Peak RAM)**: Examples should not exceed 2GB of RAM to ensure compatibility with typical beginner systems.
-   **Iterations/Steps Per Second (Throughput Rate)**: Where applicable (e.g., for iterative simulations), provide guidance on expected throughput.
-   **Scalability (Performance vs. Input Size)**: Briefly discuss how performance might change with increased complexity for advanced learners, without requiring complex scaling examples.

**Alternatives Considered**:
-   *Requiring high-performance benchmarks*: Rejected, as it would alienate the target audience with limited hardware.
-   *Providing no performance guidance*: Rejected, as this could lead to frustrating experiences for learners if examples perform poorly on their machines.
