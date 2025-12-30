# Implementation Plan: Update Docusaurus Homepage Feature Image

**Branch**: `2-docusaurus-feature-image` | **Date**: 2025-12-24 | **Spec**: [specs/2-docusaurus-feature-image/spec.md](../spec.md)
**Input**: Feature specification from `/specs/2-docusaurus-feature-image/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Replace the first feature SVG (`undraw_docusaurus_mountain.svg`) with JPEG `embodied_intelligence.jpeg` below the hero, preserving layout and responsiveness. The implementation will modify the `src/components/HomepageFeatures/index.tsx` file to replace the SVG component with an img tag while maintaining the same CSS class and styling.

## Technical Context

**Language/Version**: TypeScript/JavaScript for Docusaurus v3+
**Primary Dependencies**: Docusaurus framework, React, CSS modules
**Storage**: Static image file at `static/img/embodied_intelligence.jpeg`
**Testing**: Manual verification in browser
**Target Platform**: Web (all modern browsers)
**Project Type**: Web documentation site
**Performance Goals**: Maintain same load times and responsiveness as original SVG
**Constraints**: Must preserve existing `featureSvg` CSS class and layout properties
**Scale/Scope**: Single component modification affecting homepage feature section

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- Code quality: Follow existing code patterns and TypeScript conventions
- Security: No security implications for static image replacement
- Performance: Ensure image optimization for web delivery
- Maintainability: Preserve existing component structure and patterns

## Project Structure

### Documentation (this feature)

```text
specs/2-docusaurus-feature-image/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── components/
│   └── HomepageFeatures/
│       ├── index.tsx          # Modified to replace SVG with JPEG
│       └── styles.module.css  # Contains featureSvg class (unchanged)
└── static/
    └── img/
        └── embodied_intelligence.jpeg  # Target image file
```

**Structure Decision**: Single component modification in existing Docusaurus project structure. The implementation will replace the SVG component with an img tag in the existing HomepageFeatures component while preserving all other functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [No violations identified] | [All requirements met within standard practices] |