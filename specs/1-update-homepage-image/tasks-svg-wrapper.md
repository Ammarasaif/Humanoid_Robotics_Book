---
description: "Task list for updating homepage feature image using SVG wrapper for embodied_intelligence.jpeg"
---

# Tasks: Update Homepage Feature Image with SVG Wrapper

**Input**: Design documents from `/specs/1-update-homepage-image/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Verify Docusaurus project structure and dependencies
- [x] T002 Confirm new image exists at static/img/embodied_intelligence.jpeg
- [x] T003 [P] Backup current HomepageFeatures component in src/components/HomepageFeatures/index.tsx

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Examine current HomepageFeatures component structure in src/components/HomepageFeatures/index.tsx
- [x] T005 [P] Identify "Embodied Intelligence" feature in FeatureList array in src/components/HomepageFeatures/index.tsx
- [x] T006 [P] Document current image dimensions and CSS styling in src/components/HomepageFeatures/styles.module.css

---

## Phase 3: User Story 1 - Create SVG Wrapper (Priority: P1) 🎯 MVP

**Goal**: Replace the default feature image in the "Embodied Intelligence" section with an SVG wrapper that references the new "embodied_intelligence.jpeg" image while maintaining exact same dimensions and preserving all existing layout styles and text

**Independent Test**: The homepage displays the new "embodied_intelligence.jpeg" image through an SVG wrapper in the "Embodied Intelligence" feature section without any visual distortions, layout changes, or broken image links

### Implementation for User Story 1

- [x] T007 [US1] Create new SVG wrapper file at static/img/undraw_docusaurus_mountain.svg with embedded image reference
- [x] T008 [US1] Verify SVG wrapper has correct viewBox and image dimensions
- [x] T009 [US1] Test image rendering in development environment
- [x] T010 [US1] Validate that title and description remain unchanged in src/components/HomepageFeatures/index.tsx

---

## Phase 4: User Story 2 - Maintain Consistent Layout (Priority: P1)

**Goal**: Ensure the feature section layout remains consistent so that users don't experience any visual disruptions or unexpected changes in the page structure

**Independent Test**: The feature section maintains the same visual appearance and styling as before, with only the image being updated

### Implementation for User Story 2

- [x] T011 [US2] Verify layout consistency with other feature sections in src/components/HomepageFeatures/index.tsx
- [x] T012 [US2] Test responsive behavior on different screen sizes
- [x] T013 [US2] Validate CSS classes remain unchanged in src/components/HomepageFeatures/styles.module.css
- [x] T014 [US2] Confirm no visual regressions in other feature sections in src/components/HomepageFeatures/index.tsx

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T015 [P] Test homepage functionality after image update
- [x] T016 Run build to ensure no errors
- [x] T017 Verify image loads correctly in all supported browsers
- [x] T018 Validate accessibility of new image
- [x] T019 Update any documentation if needed

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
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

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
4. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
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