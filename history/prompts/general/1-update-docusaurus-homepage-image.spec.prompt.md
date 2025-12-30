---
id: 1
title: Update Docusaurus Homepage Feature Image
stage: spec
date: 2025-12-24
surface: agent
model: Claude Sonnet 4.5
feature: docusaurus-feature-image
branch: 2-docusaurus-feature-image
user: ESHOP
command: /sp.specify
labels: [docusaurus, homepage, image]
links:
  spec: specs/2-docusaurus-feature-image/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/2-docusaurus-feature-image/spec.md
  - specs/2-docusaurus-feature-image/checklists/requirements.md
tests:
  - null
---

## Prompt

```text

/sp.specify
 Update Docusaurus homepage feature image below the hero

Target audience: Developers/authors using Docusaurus default UI for book projects

Focus: Replace default SVG (`undraw_docusaurus_mountain.svg`) with JPEG (`embodied_intelligence.jpeg`) safely and visibly

Success criteria:
- Feature image shows `embodied_intelligence.jpeg`
- Layout, spacing, responsiveness unchanged
- No console warnings or build errors
- Works in dev and production builds

Constraints:
- Image located at `static/img/embodied_intelligence.jpeg`
- Do not modify other features
- Preserve `featureSvg` CSS class
- Do not break Docusaurus JSX logic
- Maintain Docusaurus v3+ compatibility
- Execute only after explicit `sp.implement` command

Not building:
- Hero image changes
- Other SVGs or feature cards
- Layout, fonts, or theme modifications
```

## Response snapshot

Created feature specification for updating the Docusaurus homepage feature image. The specification includes:

- User scenarios and testing requirements
- Functional requirements for replacing the SVG with JPEG while preserving layout and CSS classes
- Success criteria with measurable outcomes
- Quality checklist to validate the specification

The spec ensures the feature image shows `embodied_intelligence.jpeg` while maintaining layout, spacing, responsiveness, and compatibility with Docusaurus v3+.

## Outcome

- ✅ Impact: Created complete feature specification for image replacement
- 🧪 Tests: null
- 📁 Files: Created spec.md and requirements.md files in specs/2-docusaurus-feature-image/
- 🔁 Next prompts: Ready for /sp.clarify or /sp.plan
- 🧠 Reflection: Specification addresses all requirements from user description

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null