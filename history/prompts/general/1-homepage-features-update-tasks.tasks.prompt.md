---
id: 1
title: Homepage-Features-Update-Tasks
stage: tasks
date: 2025-12-23
surface: agent
model: Claude Sonnet 4.5
feature: homepage-features-update
branch: master
user: ESHOP
command: sp.tasks
labels: ["homepage", "features", "update", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - tasks.md
tests: []
---

## Prompt

```
TASK
Replace the default Docusaurus Homepage Features item titled "Easy to Use" with "Embodied Intelligence" for the Physical AI & Humanoid Robotics textbook.

CONTEXT
- Framework: Docusaurus v3+ (Classic)
- Section: Homepage Features (below Hero)
- File: src/components/HomepageFeatures/index.tsx
- Tooling: Spec-Kit Plus via Claude Code
- Constraint: Do not alter layout, styles, or other feature cards.

GOAL
Update only the content of the "Easy to Use" feature to reflect Physical AI and embodied intelligence.

REQUIREMENTS
1. Change title to: "Embodied Intelligence"
2. Replace description with:
   "Learn how AI systems perceive, reason, and act in the physical world through humanoid robots."
3. Preserve existing JSX structure, formatting, and imports.
4. Ensure no TypeScript or build errors.

ACCEPTANCE
- Homepage shows "Embodied Intelligence" instead of "Easy to Use".
- No visual or functional regressions.
- Only the targeted feature entry is modified.
```

## Response snapshot

Generated tasks.md file with comprehensive task breakdown for updating the homepage features component, replacing the "Easy to Use" feature with "Embodied Intelligence" while preserving existing structure and functionality.

## Outcome

- ✅ Impact: Created comprehensive task list for implementing homepage feature update
- 🧪 Tests: N/A for task generation
- 📁 Files: Updated tasks.md with new task structure
- 🔁 Next prompts: Implementation of individual tasks
- 🧠 Reflection: Successfully adapted existing task template to new feature requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation of tasks in tasks.md