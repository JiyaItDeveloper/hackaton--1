# ADR-2: Multi-Module Documentation Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Documentation Structure" not separate ADRs for framework, format, organization).

- **Status:** Accepted
- **Date:** 2025-12-26
- **Feature:** docusaurus-module
- **Context:** Need to organize comprehensive technical documentation in a way that supports multiple distinct learning modules while maintaining navigational clarity and content separation. The documentation must serve both ROS 2 concepts and Docusaurus technical authoring in a single, cohesive site.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **Structure**: Multi-module organization with distinct topic areas
- **Navigation**: Hierarchical sidebar with clear module separation
- **Content Format**: Markdown files organized by module and chapter
- **Integration**: Single Docusaurus site hosting multiple learning paths
- **Cross-Referencing**: Limited inter-module links to maintain focus

## Consequences

### Positive

- Clear separation of concerns between different learning topics
- Scalable architecture that allows adding new modules without restructuring
- Easy for users to focus on one learning path at a time
- Maintains site cohesion while supporting diverse content types
- Enables independent development of modules
- Allows for different authors to work on separate modules
- Supports different learning journeys through module selection

### Negative

- Potential confusion for users trying to determine which module to start with
- Increased complexity in site navigation
- Risk of inconsistent content quality across modules
- Cross-module dependencies may be harder to maintain
- More complex build and deployment processes
- Potential for content duplication across modules

## Alternatives Considered

- **Single Integrated Book**: One continuous book covering both ROS 2 and Docusaurus topics - Rejected because it would create a confusing learning experience with mixed topics
- **Separate Sites**: Two completely separate Docusaurus sites - Rejected because it loses the benefit of cross-referencing and creates maintenance overhead
- **Topic-Based Organization**: Organize by technical topics rather than learning modules - Rejected because it doesn't align with pedagogical approach of structured learning paths
- **Sequential Chapters**: Linear progression from ROS 2 to Docusaurus - Rejected because users may need to reference only one topic area

## References

- Feature Spec: specs/ros2-module/spec.md
- Implementation Plan: specs/ros2-module/plan.md
- Related ADRs: ADR-1 (Documentation Architecture for ROS 2 Module)
- Evaluator Evidence: history/prompts/general/1-ros2-module-implementation.general.prompt.md