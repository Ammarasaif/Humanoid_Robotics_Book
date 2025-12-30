---
id: 7
title: Generate-tasks-img-direct
stage: tasks
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-update-homepage-image
branch: master
user: ESHOP
command: /sp.tasks
labels: [tasks, homepage, image, direct-img]
links:
  spec: ./specs/1-update-homepage-image/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-update-homepage-image/tasks-img-direct.md
tests:
  - null
---

## Prompt

TASK: Update the first Docusaurus homepage feature image below the hero to use `embodied_intelligence.jpeg` by replacing SVG rendering with <img>.

CONTEXT
- Docusaurus v3+, file: src/components/HomepageFeatures/index.tsx
- Current image: undraw_docusaurus_mountain.svg
- New image: static/img/embodied_intelligence.jpeg
- SVG wrapper approach failed.

MANDATORY RULES
1. Remove all references to the old SVG.
2. The first feature must render using <img> only.
3. Keep other features unchanged.

IMPLEMENTATION
1. Delete SVG import.
2. Replace first feature's <Svg /> with:
   <img
     src={useBaseUrl('/img/embodied_intelligence.jpeg')}
     className={styles.featureSvg}
     alt="Embodied Intelligence"
   />
3. Preserve existing CSS and layout.
4. Import useBaseUrl if needed.

CACHE RESET
1. Stop dev server.
2. Delete .docusaurus and build folders.
3. Run npm start.
4. Hard refresh browser.

VALIDATION
- First feature shows the new image.
- Old SVG is gone.
- Layout, spacing, and responsiveness are unchanged.
- No console errors.

WAIT_FOR_IMPLEMENT
- Do not execute changes until an explicit implement command is issued.

## Response snapshot

Generated comprehensive tasks for replacing the homepage feature image with a direct IMG element approach as requested by the user. The tasks follow the required format and are organized by user story:

- Phase 1: Setup tasks for project initialization
- Phase 2: Foundational tasks (blocking prerequisites)
- Phase 3: User Story 1 - Replace SVG with IMG (P1 priority)
- Phase 4: User Story 2 - Maintain consistent layout (P1 priority)
- Phase 5: Polish & Cross-Cutting Concerns

Each task follows the required format with ID, story labels, and specific file paths, enabling independent implementation and testing.

## Outcome

- ✅ Impact: Generated comprehensive task list for direct IMG implementation
- 🧪 Tests: No specific tests required for this task generation
- 📁 Files: Created specs/1-update-homepage-image/tasks-img-direct.md with proper format
- 🔁 Next prompts: Implementation of the generated tasks
- 🧠 Reflection: Tasks properly organized by user story with specific file paths

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing the generated tasks