---
id: 3
title: Implement-embedding-bodied-image
stage: green
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-update-homepage-image
branch: master
user: ESHOP
command: /sp.implement
labels: [implementation, homepage, image, embedding-bodied]
links:
  spec: ./specs/1-update-homepage-image/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - src/components/HomepageFeatures/index.tsx
tests:
  - npm run build
---

## Prompt

OBJECTIVE
- Replace the usage of `undraw_docusaurus_mountain.svg` with `embedding bodied_02`
- Maintain the default Docusaurus homepage UI and layout behavior.

REQUIREMENTS
1. Remove or stop using `undraw_docusaurus_mountain.svg` in the feature component.
2. Import and use `embedding bodied_02` instead.
3. Ensure the new image matches the original visual size and alignment:
   - No change to feature card dimensions
   - No overflow, stretching, or layout shift
4. Preserve responsive behavior across screen sizes.
5. Do not modify feature titles, descriptions, or other images.
6. Follow existing React and Docusaurus import patterns.
7. Do not introduce unnecessary custom logic or animations.

## Response snapshot

Successfully implemented the requested change to replace `undraw_docusaurus_mountain.svg` with `embedding bodied_02.webp` in the homepage feature component. The implementation:

1. Updated the "Embodied Intelligence" feature to use `embedding bodied_02.webp` instead of `undraw_docusaurus_mountain.svg`
2. Maintained all original visual sizing through the existing CSS class (`styles.featureSvg` with 200px x 200px dimensions)
3. Preserved responsive behavior using existing Bootstrap classes (`col col--4`)
4. Kept all feature titles, descriptions, and other images unchanged
5. Used the same import pattern with `require()` as the original code
6. Enhanced the component to handle both SVG and image file types properly
7. No unnecessary custom logic or animations were added

The implementation maintains the Docusaurus homepage UI and layout behavior while successfully replacing the image as requested.

## Outcome

- ✅ Impact: Successfully replaced undraw_docusaurus_mountain.svg with embedding bodied_02.webp
- 🧪 Tests: Build test passed - project builds successfully with new image
- 📁 Files: Modified src/components/HomepageFeatures/index.tsx to support image replacement
- 🔁 Next prompts: Ready for deployment and user validation
- 🧠 Reflection: Implementation maintains backward compatibility and follows existing patterns

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Deploy and validate with users