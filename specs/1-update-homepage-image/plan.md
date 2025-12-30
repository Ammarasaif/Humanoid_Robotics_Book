# Implementation Plan: Update Homepage Feature Image

**Branch**: `1-update-homepage-image` | **Date**: 2025-12-23 | **Spec**: [link to spec.md](./spec.md)
**Input**: Feature specification from `/specs/1-update-homepage-image/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Replace the default feature image in the "Embodied Intelligence" section of the Docusaurus homepage with the new "embedding_bodied_02" image from static/img while maintaining exact same dimensions and preserving all existing layout styles and text.

## Technical Context

**Language/Version**: TypeScript/JavaScript, Docusaurus v3+
**Primary Dependencies**: Docusaurus core, React, Node.js
**Storage**: N/A (static image assets)
**Testing**: N/A (UI update, visual verification)
**Target Platform**: Web browser (all supported browsers)
**Project Type**: Web
**Performance Goals**: No performance degradation, image loads efficiently
**Constraints**: Image dimensions must match original exactly to prevent layout shifts
**Scale/Scope**: Single feature section update, no user data involved

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

This change complies with the constitution by:
- Following the Spec-Driven Development process (after /sp.specify)
- Maintaining Phase Boundary Integrity (UI update within Phase 2 - Custom Book UI)
- Preserving Quality & Safety Guarantees (no breaking changes to functionality)
- Respecting Single Constitution Rule (no changes to constitution required)

## Project Structure

### Documentation (this feature)

```text
specs/1-update-homepage-image/
├── spec.md              # Feature specification
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
│       └── index.tsx    # Feature component to be updated
└── pages/
    └── index.tsx        # Homepage that uses the feature component

static/
└── img/
    └── embedding_bodied_02.webp  # New image to be used
```

**Structure Decision**: This feature modifies existing Docusaurus components to update the feature image while maintaining all existing functionality and layout.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|