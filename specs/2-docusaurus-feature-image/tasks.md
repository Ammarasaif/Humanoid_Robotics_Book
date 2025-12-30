# Implementation Tasks: Update Docusaurus Homepage Feature Image

**Feature**: Update Docusaurus Homepage Feature Image
**Branch**: 2-docusaurus-feature-image
**Created**: 2025-12-24
**Plan**: [specs/2-docusaurus-feature-image/plan.md](./plan.md)

## Implementation Strategy

Implement the feature in priority order, starting with the core functionality (User Story 1), then visual consistency (User Story 2), and finally build validation (User Story 3). Each user story should be independently testable with its own verification steps.

## Dependencies

- User Story 1 (P1) must be completed before User Story 2 (P1) and User Story 3 (P2)
- User Story 2 and User Story 3 can be developed in parallel after User Story 1 completion

## Parallel Execution Examples

- User Story 2 (visual consistency) and User Story 3 (build validation) can be worked on in parallel after User Story 1 is complete
- Testing tasks can be executed in parallel with implementation tasks

## Phase 1: Setup

- [X] T001 Set up development environment and verify project structure
- [X] T002 Confirm target image exists at `static/img/embodied_intelligence.jpg`
- [X] T003 Verify Docusaurus development server can be started with `npm run start`

## Phase 2: Foundational Tasks

- [X] T004 Verify current homepage feature functionality by running development server
- [X] T005 Locate and examine `src/components/HomepageFeatures/index.tsx` file
- [X] T006 Locate and examine `src/components/HomepageFeatures/styles.module.css` file

## Phase 3: User Story 1 - Replace Default Feature Image (Priority: P1)

**Goal**: Replace the first feature SVG (`undraw_docusaurus_mountain.svg`) with JPEG `embodied_intelligence.jpg` below the hero.

**Independent Test**: Can be fully tested by visiting the homepage and verifying that the `embodied_intelligence.jpg` image is displayed in place of the default mountain SVG, with the same layout and spacing preserved.

- [X] T007 [US1] Replace SVG import with img tag in `SimpleFeatureComponent` function in `src/components/HomepageFeatures/index.tsx`
- [X] T008 [US1] Update `SimpleFeatureComponent` to use `useBaseUrl` for image path in `src/components/HomepageFeatures/index.tsx`
- [X] T009 [US1] Add appropriate alt text to the img tag in `src/components/HomepageFeatures/index.tsx`
- [X] T010 [US1] Verify `featureSvg` CSS class is preserved on the new img element in `src/components/HomepageFeatures/index.tsx`
- [X] T011 [US1] Test that the new image displays correctly in development server

## Phase 4: User Story 2 - Maintain Visual Consistency (Priority: P1)

**Goal**: Ensure the replaced image maintains the same layout, spacing, and responsiveness as the original.

**Independent Test**: Can be tested by checking that the layout, spacing, and responsiveness remain unchanged after the image replacement, with no visual regressions.

- [X] T012 [US2] Verify layout dimensions remain unchanged after image replacement
- [X] T013 [US2] Test responsiveness on different screen sizes (desktop, tablet, mobile)
- [X] T014 [US2] Verify CSS class `featureSvg` maintains 200px height/width properties
- [X] T015 [US2] Check that other features remain unchanged and properly aligned
- [X] T016 [US2] Validate that spacing between features is preserved

## Phase 5: User Story 3 - Ensure No Build Errors (Priority: P2)

**Goal**: Ensure the image replacement does not introduce any console warnings or build errors.

**Independent Test**: Can be tested by building the site and verifying that there are no console warnings or build errors related to the image replacement.

- [X] T017 [US3] Test development build with `npm run start` and check for errors
- [X] T018 [US3] Test production build with `npm run build` and check for errors
- [X] T019 [US3] Open browser console and verify no image-related warnings or errors
- [X] T020 [US3] Verify image loads properly in production build
- [X] T021 [US3] Confirm Docusaurus v3+ compatibility is maintained

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T022 Clear browser cache and Docusaurus cache (delete `.docusaurus` directory)
- [X] T023 Perform final verification in development and production builds
- [X] T024 Document any changes made for future reference
- [X] T025 Clean up any temporary files or test artifacts
- [X] T026 Final acceptance testing: verify all acceptance scenarios from spec are met