---
id: 4
title: Update Homepage Image Feature Tasks
stage: tasks
date: 2025-12-24
surface: agent
model: Claude Sonnet 4.5
feature: docusaurus-feature-image
branch: 2-docusaurus-feature-image
user: ESHOP
command: /sp.tasks
labels: [docusaurus, homepage, image, tasks]
links:
  spec: specs/2-docusaurus-feature-image/spec.md
  plan: specs/2-docusaurus-feature-image/plan.md
  tasks: specs/2-docusaurus-feature-image/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/2-docusaurus-feature-image/tasks.md
tests:
  - null
---

## Prompt

```text

TASK: Update the "Embodied Intelligence" homepage feature image below the hero in Docusaurus.

CONTEXT:
- File: src/components/HomepageFeatures/index.tsx
- Current: undraw_docusaurus_mountain.svg (React SVG)
- New: embodied_intelligence.jpg in static/img
- Issue: SVG rendering prevents JPEG display

STEPS:
1. Remove import/reference to `undraw_docusaurus_mountain.svg`.
2. Replace <Svg /> for "Embodied Intelligence" with:
   <img
     src={useBaseUrl('/img/embodied_intelligence.jpg')}
     className={styles.featureSvg}
     alt="Embodied Intelligence"
   />
3. Import `useBaseUrl` if missing.
4. Keep other features unchanged.
5. Preserve `featureSvg` CSS class.
6. Stop dev server, delete `.docusaurus` and `build`.
7. Restart dev server and hard refresh (Ctrl+Shift+R).

VALIDATION:
- Feature displays `embodied_intelligence.jpg`.
- Old SVG removed.
- Layout, spacing, responsiveness intact.
- No console errors.

EXECUTION:
- Apply only after explicit `implement` command.
```

## Response snapshot

Updated the tasks file for the homepage feature image update. The implementation has already been completed where the SVG was replaced with an img tag using the correct `embodied_intelligence.jpg` file. All tasks in the tasks.md file have been marked as completed, reflecting that the image replacement has been successfully implemented in `src/components/HomepageFeatures/index.tsx`. The `useBaseUrl` hook is properly used for the image path, the `featureSvg` CSS class is preserved, and appropriate alt text is included.

## Outcome

- ✅ Impact: All tasks completed for homepage feature image update
- 🧪 Tests: null
- 📁 Files: Updated tasks.md in specs/2-docusaurus-feature-image/
- 🔁 Next prompts: Ready for final verification
- 🧠 Reflection: Implementation successfully completed with correct image format

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null