---
id: 5
title: Generate-tasks-svg-wrapper
stage: tasks
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-update-homepage-image
branch: master
user: ESHOP
command: /sp.tasks
labels: [tasks, homepage, image, svg-wrapper]
links:
  spec: ./specs/1-update-homepage-image/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-update-homepage-image/tasks-svg-wrapper.md
tests:
  - null
---

## Prompt

TASK: Replace the default Docusaurus homepage feature image below the hero with `embodied_intelligence.jpeg` using an SVG wrapper.

CONTEXT
- Project: Docusaurus v3+ default UI
- File: src/components/HomepageFeatures/index.tsx
- Current feature image: undraw_docusaurus_mountain.svg
- New image: static/img/embodied_intelligence.jpeg
- Goal: ensure the image updates successfully without modifying JSX rendering logic.

REQUIRED IMPLEMENTATION
1. Create a new SVG wrapper file:
   static/img/undraw_docusaurus_mountain.svg
2. Paste the following inside the SVG:
   <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 800 600">
     <image href="./embodied_intelligence.jpeg" width="800" height="600" />
   </svg>
3. Keep the JPEG file in:
   static/img/embodied_intelligence.jpeg
4. Do NOT modify JSX or feature logic in HomepageFeatures.
5. Preserve all CSS classes and responsiveness.

VALIDATION
- Stop dev server completely.
- Delete `.docusaurus` and `build` directories.
- Run `npm run start`.
- Hard refresh browser (Ctrl + Shift + R).
- Verify the new image appears below the hero.

SUCCESS CRITERIA
- Old SVG is replaced with the SVG wrapper referencing `embodied_intelligence.jpeg`.
- Layout, spacing, and responsiveness remain intact.
- No console warnings or errors.

WAIT_FOR_IMPLEMENT
- Do not execute any file changes until an explicit `implement` command is issued.

## Response snapshot

Generated comprehensive tasks for replacing the homepage feature image with an SVG wrapper that references the new embodied_intelligence.jpeg image. The tasks follow the required format and are organized by user story:

- Phase 1: Setup tasks for project initialization
- Phase 2: Foundational tasks (blocking prerequisites)
- Phase 3: User Story 1 - Create SVG wrapper (P1 priority)
- Phase 4: User Story 2 - Maintain consistent layout (P1 priority)
- Phase 5: Polish & Cross-Cutting Concerns

Each task follows the required format with ID, story labels, and specific file paths, enabling independent implementation and testing.

## Outcome

- ✅ Impact: Generated comprehensive task list for SVG wrapper implementation
- 🧪 Tests: No specific tests required for this task generation
- 📁 Files: Created specs/1-update-homepage-image/tasks-svg-wrapper.md with proper format
- 🔁 Next prompts: Implementation of the generated tasks
- 🧠 Reflection: Tasks properly organized by user story with specific file paths

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing the generated tasks