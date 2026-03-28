# Research Plan: Navbar & Footer — "Physical AI & Humanoid Robotics"

**Feature Branch**: `001-navbar-footer-design` | **Date**: December 10, 2025
**Purpose**: Address key design decisions and unknowns identified during planning for the Navbar and Footer.

## Research Tasks

### 1. Navbar Structure: Top-Level vs. Dropdowns

**Objective**: Determine the optimal structure for the navbar to ensure clear, intuitive navigation between book modules and subchapters, considering Docusaurus capabilities.
**Context**: The spec requires showing all modules (Introduction, Module 1–4, Capstone) and providing dropdowns for subchapters where applicable, while avoiding clutter.
**Questions to Answer**:
- What are Docusaurus's native capabilities and best practices for creating multi-level navigation (top-level items with dropdowns)?
- How many subchapters per module warrant a dropdown versus direct top-level links for clarity?
- What are the implications for user experience (e.g., discoverability, click-through rates) for each approach?
**Expected Outcome**: A clear recommendation for navbar structure (e.g., which modules are top-level, which use dropdowns, and maximum items per dropdown).

### 2. Footer Content Selection

**Objective**: Finalize the specific content for the footer, ensuring it includes all required metadata and links without being excessive.
**Context**: The spec requires copyright, author info, resources, and social links. The "Edge Cases" section in `spec.md` also raises questions about external links and specific content for author/resources.
**Questions to Answer**:
- What specific resources (e.g., links to external tools, glossary, index) should be included in the footer's "resources" section?
- How should "author contact/info" be presented (e.g., email address, link to author page, social media profile)?
- What social media links (if any) are most relevant for the book's audience?
**Expected Outcome**: A detailed list of all content items and their corresponding links (if applicable) for the footer.

### 3. Styling: Color, Font, Spacing Consistency

**Objective**: Research Docusaurus's styling customization options and best practices to ensure the navbar and footer align with the book's branding and maintain a minimalist aesthetic.
**Context**: The spec emphasizes minimalism, consistency with branding, and avoiding clutter.
**Questions to Answer**:
- How does Docusaurus handle theme customization for navbar and footer components (e.g., CSS variables, custom CSS)?
- What color palettes, font choices, and spacing guidelines align with a minimalist and educational book aesthetic?
- Are there existing Docusaurus themes or plugins that provide a good starting point for the desired aesthetic?
**Expected Outcome**: A set of styling guidelines (colors, fonts, spacing) and the technical approach for implementing them within Docusaurus.

### 4. Mobile vs. Desktop Tradeoffs (Responsiveness)

**Objective**: Identify specific design and functionality tradeoffs required to ensure optimal responsiveness and user-friendliness of the navbar and footer across mobile and desktop devices.
**Context**: The spec requires full responsiveness and user-friendliness on both platforms. The "Edge Cases" section in `spec.md` mentions the need for intuitive mobile navigation.
**Questions to Answer**:
- What are standard Docusaurus practices for responsive navbar design (e.g., hamburger menus, collapsing elements)?
- How should the footer content be re-arranged or prioritized on smaller screens to prevent clutter?
- Are there specific breakpoints or design patterns that should be applied for different device widths?
**Expected Outcome**: Design recommendations for responsive behavior of the navbar (e.g., mobile menu interaction) and footer (e.g., stacked content).

## Next Steps

Upon completion of this research, the findings will be consolidated into a "Decisions" section within `research.md`, and the plan will proceed to Phase 1: Design & Contracts.

---

## Decisions

This section summarizes the decisions made based on the research conducted in Phase 1. These decisions will guide the implementation of the Navbar and Footer.

### 1. Navbar Structure: Top-Level vs. Dropdowns

**Decision**: All main modules (Introduction, Module 1-4, Capstone) will be top-level links. Subchapters will be accessible via Docusaurus's native sidebar navigation within each module's landing page. No dropdowns will be used in the top navbar itself to maintain minimalism and avoid clutter.
**Rationale**: Docusaurus's sidebar is designed for detailed chapter navigation. Adding complex dropdowns to the top navbar would increase visual clutter, especially on mobile, and potentially conflict with existing Docusaurus navigation patterns. Top-level links for modules provide clear, direct access to major sections.
**Alternatives Considered**: Implementing custom JavaScript dropdowns (adds complexity, deviates from Docusaurus native functionality), having all subchapters as top-level links (causes excessive clutter).

### 2. Footer Content Selection

**Decision**:
- **Copyright**: Dynamic copyright year "© [Current Year] [Book Title]".
- **Author Info**: Link to a dedicated "About the Authors" page within the book, or a short static text mentioning key contributors. For now, a simple text "By [Author Name]" will suffice.
- **Resources**: A link to a "References" page (collecting all APA citations used throughout the book) and a "Glossary" page.
- **Social Links**: Links to GitHub repository (for code examples/contributions) and potentially a relevant academic/professional social platform (e.g., LinkedIn for authors). For minimalism, start with GitHub only.
**Rationale**: Balances required information with minimalism. Centralizing author info and resources on dedicated pages maintains a clean footer. GitHub is crucial for a technical book.
**Alternatives Considered**: Direct email in footer (spam risk), extensive list of external links (clutter).

### 3. Styling: Color, Font, Spacing Consistency

**Decision**: Utilize Docusaurus's theming capabilities (CSS variables) for primary styling. Custom CSS (`src/css/custom.css`) will be used sparingly for fine-tuning as needed, adhering to a minimalist aesthetic that prioritizes readability.
**Rationale**: Leveraging Docusaurus's built-in theme system ensures consistency and maintainability. Minimalist design aligns with book branding, focusing reader attention on content.
**Alternatives Considered**: Extensive custom CSS (potential for inconsistency, harder to maintain), using a heavily customized external theme (might introduce unexpected conflicts).

### 4. Mobile vs. Desktop Tradeoffs (Responsiveness)

**Decision**: Docusaurus's default responsive navbar behavior (e.g., hamburger menu on mobile) will be adopted. For the footer, content will stack vertically on mobile screens.
**Rationale**: Docusaurus provides a robust, pre-built responsive layout. Stacking footer content on mobile ensures readability without horizontal scrolling.
**Alternatives Considered**: Custom mobile navigation (unnecessary complexity), hiding certain footer elements on mobile (might remove essential information).

