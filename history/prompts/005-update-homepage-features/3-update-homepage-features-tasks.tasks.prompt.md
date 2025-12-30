---
id: 3
title: update-homepage-features-tasks
stage: tasks
date: 2025-12-25
surface: agent
model: Claude Sonnet 4.5
feature: update-homepage-features
branch: 005-update-homepage-features
user: ESHOP
command: /sp.tasks
labels: ["tasks", "homepage", "features", "docusaurus", "ui"]
links:
  spec: "specs/005-update-homepage-features/spec.md"
  plan: "specs/005-update-homepage-features/plan.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/005-update-homepage-features/tasks.md"
tests: []
prompt_text: |

  TASK 1: Update FeatureList
  - Open src/components/HomepageFeatures/index.tsx
  - Replace FeatureList array with:

  Feature 1:
  title: "Autonomous Systems"
  description: "Intelligent agents capable of independent perception, decision-making, and action in complex real-world environments."
  image: "autonomous_system.jpg"  (or .png)

  Feature 2:
  title: "Cognitive Embodiment"
  description: "Human-inspired systems integrating physical bodies with cognitive processes to learn, adapt, and interact naturally."
  image: "cognitive_Embodiment.jpeg"  (or .png)

  Feature 3:
  title: "Real-World Intelligence"
  description: "AI systems designed for practical tasks, seamlessly bridging perception, reasoning, and actuation in dynamic settings."
  image: "real_world_intelligence.webp"  (or .png)

  TASK 2: Replace SVG logic
  - Remove Svg props/components
  - Use <img> with proper import:
  <img
    src={require('@site/static/img/<image>').default}
    className={styles.featureImage}
    alt="<title>"
  />

  TASK 3: CSS enforcement
  - Edit src/components/HomepageFeatures/styles.module.css:
  .featureImage {
    max-width: 200px;
    width: 100%;
    height: auto;
    display: block;
    margin: 0 auto;
  }

  TASK 4: Validation
  - Homepage renders all three features with updated titles, descriptions, and JPG/PNG/WBP/JPEG images
  - No old SVG references remain
  - Layout and responsiveness intact
  - Build passes with zero errors

response_text: |
  Successfully created task list for updating homepage features with Physical AI/Humanoid Robotics content.
  Created tasks file at specs/005-update-homepage-features/tasks.md
  Tasks organized by user story with proper dependencies and parallel execution opportunities:
  - Phase 1: Setup tasks
  - Phase 2: Foundational tasks
  - Phase 3: User Story 1 - Update Homepage Features Content (P1)
  - Phase 4: User Story 2 - Maintain Visual Design and Responsiveness (P2)
  - Phase 5: User Story 3 - Ensure Image Loading and Format Support (P3)
  - Phase 6: Polish & Cross-Cutting Concerns
  All tasks follow the required checklist format with IDs, story labels, and file paths.

outcome: "Tasks created successfully with proper structure and dependencies"
evaluation: "The tasks file meets all quality criteria and is ready for implementation"
---

---
description: "Task list for updating homepage features with Physical AI/Humanoid Robotics content"
---

# Tasks: Update Homepage Features for Physical AI/Humanoid Robotics Book

**Input**: Design documents from `/specs/005-update-homepage-features/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No explicit tests requested in feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `src/`, `static/` at repository root
- **Component**: `src/components/HomepageFeatures/`
- **Styles**: `src/components/HomepageFeatures/styles.module.css`
- **Images**: `static/img/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Verify project structure and dependencies for Docusaurus v3+
- [ ] T002 [P] Verify required images exist in static/img/ directory
- [ ] T003 [P] Verify src/components/HomepageFeatures/index.tsx exists

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T004 Verify current HomepageFeatures component structure and functionality
- [ ] T005 [P] Update CSS to support new image styling requirements in src/components/HomepageFeatures/styles.module.css
- [ ] T006 [P] Prepare image imports for all three new features in src/components/HomepageFeatures/index.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Update Homepage Features Content (Priority: P1) 🎯 MVP

**Goal**: Replace default Docusaurus homepage features with academic-level content for Physical AI/Humanoid Robotics book featuring "Autonomous Systems", "Cognitive Embodiment", and "Real-World Intelligence" with corresponding images.

**Independent Test**: Can be fully tested by visiting the homepage and verifying that the three features display the correct academic-level titles, descriptions, and images instead of the default Docusaurus features.

### Implementation for User Story 1

- [ ] T007 [P] [US1] Import autonomous_system.jpg in src/components/HomepageFeatures/index.tsx
- [ ] T008 [P] [US1] Import cognitive_Embodiment.jpeg in src/components/HomepageFeatures/index.tsx
- [ ] T009 [P] [US1] Import real_world_intelligence.webp in src/components/HomepageFeatures/index.tsx
- [ ] T010 [US1] Replace FeatureList array with new academic content in src/components/HomepageFeatures/index.tsx
- [ ] T011 [US1] Update first feature to "Autonomous Systems" with academic description and image in src/components/HomepageFeatures/index.tsx
- [ ] T012 [US1] Update second feature to "Cognitive Embodiment" with academic description and image in src/components/HomepageFeatures/index.tsx
- [ ] T013 [US1] Update third feature to "Real-World Intelligence" with academic description and image in src/components/HomepageFeatures/index.tsx
- [ ] T014 [US1] Remove all SVG imports and references in src/components/HomepageFeatures/index.tsx
- [ ] T015 [US1] Update image rendering logic to use <img> tags with proper className and alt attributes in src/components/HomepageFeatures/index.tsx

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Maintain Visual Design and Responsiveness (Priority: P2)

**Goal**: Ensure the updated features maintain the same layout, spacing, and responsive behavior as the original Docusaurus features for consistent user experience across devices.

**Independent Test**: Can be fully tested by viewing the homepage on different screen sizes and verifying that the layout, spacing, and responsiveness match the default Docusaurus UI standards.

### Implementation for User Story 2

- [ ] T016 [US2] Verify CSS maintains proper max-width constraints for images in src/components/HomepageFeatures/styles.module.css
- [ ] T017 [US2] Ensure images maintain responsive behavior with auto height in src/components/HomepageFeatures/styles.module.css
- [ ] T018 [US2] Verify centered alignment of images in feature cards in src/components/HomepageFeatures/styles.module.css
- [ ] T019 [US2] Test responsive behavior on desktop, tablet, and mobile in src/components/HomepageFeatures/index.tsx
- [ ] T020 [US2] Validate layout consistency across different screen sizes

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Ensure Image Loading and Format Support (Priority: P3)

**Goal**: Ensure all academic-level feature images load correctly in supported formats (PNG, JPG, WebP) for optimal visual presentation of content.

**Independent Test**: Can be fully tested by verifying that all three feature images load correctly without errors and display the appropriate academic content.

### Implementation for User Story 3

- [ ] T021 [US3] Verify all three image formats (JPG, JPEG, WebP) load correctly in src/components/HomepageFeatures/index.tsx
- [ ] T022 [US3] Confirm alt text attributes are properly set for accessibility in src/components/HomepageFeatures/index.tsx
- [ ] T023 [US3] Test image loading fallback behavior in src/components/HomepageFeatures/index.tsx
- [ ] T024 [US3] Validate image paths and references in src/components/HomepageFeatures/index.tsx
- [ ] T025 [US3] Verify no broken image links occur in src/components/HomepageFeatures/index.tsx

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T026 [P] Run Docusaurus build to verify no errors or warnings
- [ ] T027 [P] Test homepage rendering across different browsers
- [ ] T028 [P] Validate accessibility features (alt text, semantic structure)
- [ ] T029 [P] Run quickstart.md validation steps
- [ ] T030 [P] Verify all acceptance scenarios from spec.md pass

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on US1 implementation
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Depends on US1 implementation

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all image imports for User Story 1 together:
Task: "Import autonomous_system.jpg in src/components/HomepageFeatures/index.tsx"
Task: "Import cognitive_Embodiment.jpeg in src/components/HomepageFeatures/index.tsx"
Task: "Import real_world_intelligence.webp in src/components/HomepageFeatures/index.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence