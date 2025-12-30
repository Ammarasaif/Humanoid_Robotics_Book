---
id: 5
title: Restore-Docusaurus-Homepage-Fix-Exports
stage: green
date: 2025-12-23
surface: agent
model: Claude Sonnet 4.5
feature: docusaurus-homepage-fix
branch: master
user: ESHOP
command: sp.implement
labels: ["docusaurus", "exports", "es-modules", "homepage", "runtime-error"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - src/components/HomepageHero/index.js
  - src/components/HomepageHero/index.tsx
  - tasks.md
tests: []
---

## Prompt

```
SPECIFY: Restore the Docusaurus homepage and resolve the runtime error "exports is not defined" in src/components/HomepageHero/index.js. The page is currently crashing due to CommonJS exports in an ES module environment.

PLAN:
1. Analyze src/components/HomepageHero/index.js for any `exports` or `module.exports` usage.
2. Convert all exports to ES module syntax (`export` / `export default`).
3. Ensure all imports in src/pages/index.tsx and other affected files use ES module `import` statements.
4. Preserve all existing functionality, animations, and layout of the HomepageHero component.
5. Ensure the homepage loads correctly without crashing.

TASK: Using Spec-Driven Development with Claude Code and SpeckitPlus, provide corrected code for HomepageHero/index.js and any necessary changes in index.tsx to fully restore the homepage and remove the runtime error.
```

## Response snapshot

Successfully implemented the fix for the Docusaurus runtime error "exports is not defined" in src/components/HomepageHero/index.js. The issue was caused by having both a problematic index.js file with CommonJS exports and a correct index.tsx file with ES module syntax. The solution was to remove the problematic index.js file that contained CommonJS syntax:

- Object.defineProperty(exports, "__esModule", { value: true }) on line 37
- exports.default = HomepageHero on line 146

The src/components/HomepageHero/index.tsx file already had the correct ES module syntax with proper import statements and export default HomepageHero. After removing the problematic file, the build completed successfully and the homepage loads without runtime errors. All functionality, animations, and layout of the HomepageHero component are preserved.

## Outcome

- ✅ Impact: Fixed "exports is not defined" runtime error in Docusaurus homepage
- 🧪 Tests: Build process completed successfully with no errors
- 📁 Files: Removed src/components/HomepageHero/index.js, preserved index.tsx with ES modules
- 🔁 Next prompts: Homepage loads correctly with particle animations and all functionality
- 🧠 Reflection: Resolved CommonJS vs ES module conflict by removing duplicate file

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Build successful, runtime error resolved
- Prompt variant (if applicable): CommonJS/ES module compatibility issue
- Next experiment (smallest change to try): Deploy the fixed implementation