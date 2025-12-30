---
id: 2
title: Update Docusaurus Homepage Feature Image Plan
stage: plan
date: 2025-12-24
surface: agent
model: Claude Sonnet 4.5
feature: docusaurus-feature-image
branch: 2-docusaurus-feature-image
user: ESHOP
command: /sp.plan
labels: [docusaurus, homepage, image, plan]
links:
  spec: specs/2-docusaurus-feature-image/spec.md
  plan: specs/2-docusaurus-feature-image/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/2-docusaurus-feature-image/plan.md
  - specs/2-docusaurus-feature-image/research.md
  - specs/2-docusaurus-feature-image/data-model.md
  - specs/2-docusaurus-feature-image/quickstart.md
  - specs/2-docusaurus-feature-image/architecture-validation.md
  - specs/2-docusaurus-feature-image/checklists/plan-quality.md
tests:
  - null
---

## Prompt

```text

/sp.plan
Update Docusaurus homepage feature image

Objective: Replace the first feature SVG (`undraw_docusaurus_mountain.svg`) with JPEG `embodied_intelligence.jpeg` below the hero, preserving layout and responsiveness.

Steps:
1. Confirm `static/img/embodied_intelligence.jpeg` exists.
2. Open `src/components/HomepageFeatures/index.tsx`.
3. Locate the first feature using the old SVG.
4. Remove SVG import and `<Svg />` usage.
5. Add:
   <img
     src={useBaseUrl('/img/embodied_intelligence.jpeg')}
     className={styles.featureSvg}
     alt="Embodied Intelligence"
   />
6. Import `useBaseUrl` if missing.
7. Keep other features and SVGs unchanged.
8. Retain `featureSvg` CSS class.
9. Stop dev server, delete `.docusaurus` and `build`.
10. Restart dev server and hard refresh (Ctrl+Shift+R).
11. Verify new image displays with correct layout and no console errors.

Validation:
- First feature shows `embodied_intelligence.jpeg`.
- Old SVG gone; other features unchanged.
- Layout, spacing, responsiveness intact.
- Works in dev and production builds.

Constraints:
- Do not modify hero or other features.
- Do not change fonts, theme, or global layout.
- Execute only after explicit `sp.implement` command.
```

## Response snapshot

Created comprehensive implementation plan for updating the Docusaurus homepage feature image. The plan includes:

- Detailed technical approach for replacing SVG with JPEG
- Research document analyzing current state
- Data model documenting component structure
- Quickstart guide with implementation steps
- Architecture validation confirming no critical risks
- Quality checklist validating the plan completeness

The implementation will modify `src/components/HomepageFeatures/index.tsx` to replace the SVG component with an img tag while preserving the `featureSvg` CSS class and all layout properties.

## Outcome

- ✅ Impact: Created complete implementation plan with supporting documentation
- 🧪 Tests: null
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, architecture-validation.md, and plan-quality.md
- 🔁 Next prompts: Ready for /sp.tasks to generate implementation tasks
- 🧠 Reflection: Plan addresses all requirements and follows Docusaurus best practices

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null