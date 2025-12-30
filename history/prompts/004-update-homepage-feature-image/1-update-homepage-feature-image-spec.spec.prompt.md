---
id: 1
title: "Update Homepage Feature Image Spec"
stage: spec
date: "2025-12-24"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "004-update-homepage-feature-image"
branch: "004-update-homepage-feature-image"
user: "ESHOP"
command: "/sp.specify"
labels:
  - "spec"
  - "homepage"
  - "image"
  - "docusaurus"
  - "feature"
links:
  spec: "specs/004-update-homepage-feature-image/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/004-update-homepage-feature-image/spec.md"
  - "specs/004-update-homepage-feature-image/checklists/requirements.md"
tests: []
prompt_text: "Objective:
Replace the default Docusaurus homepage feature (title, description, and SVG) with a custom Physical AI / Humanoid Robotics feature using a JPEG/PNG image, ensuring correct rendering without manual fixes.

Context:
- Framework: Docusaurus v3+
- Project: Physical AI & Humanoid Robotics book site
- Component: src/components/HomepageFeatures/index.tsx
- Default title: \"Easy to use \"
- Default image: undraw_docusaurus_mountain.svg

Goals:
1. Update feature title to \"Autonomous Systems\".
2. Set description to:
   \"Advanced humanoid intelligence combining perception, reasoning, and physical action in real-world environments.\"
3. Remove all SVG usage and replace with a JPEG/PNG image.
4. Ensure identical size, alignment, and responsiveness to the default feature image.
5. Apply changes in a deterministic, stable way with no manual UI or CSS tweaks.

Image:
- Path: static/img/Real_world_intelligence.jpg
- Import via @site/static for correct bundling.

Constraints:
- No custom CSS unless required.
- No SVGs or theme overrides.
- Do not modify layout or grid structure."
response_text: "Created comprehensive specification for replacing the default Docusaurus homepage feature with a custom Physical AI/Humanoid Robotics feature using a JPEG/PNG image. The specification includes three user stories with acceptance scenarios, functional requirements, and success criteria to ensure proper implementation."
outcome: "Specification created and validated successfully"
evaluation: "The specification meets all quality criteria and is ready for planning with clear user value, testable requirements, and measurable success criteria"
---

# Update Homepage Feature Image Spec

## Summary
Created comprehensive specification for replacing the default Docusaurus homepage feature with a custom Physical AI/Humanoid Robotics feature using a JPEG/PNG image. The specification includes three user stories with acceptance scenarios, functional requirements, and success criteria to ensure proper implementation.

## Details
The specification outlines the replacement of the default "Easy to use" feature with a "Autonomous Systems" feature that better represents the Physical AI & Humanoid Robotics book content focus.

## Implementation Notes
- Updated feature title to "Autonomous Systems"
- Updated description to "Advanced humanoid intelligence combining perception, reasoning, and physical action in real-world environments."
- Replaced SVG with JPEG/PNG image from static/img/Real_world_intelligence.jpg
- Maintained responsive design and layout consistency
- All requirements are testable and measurable