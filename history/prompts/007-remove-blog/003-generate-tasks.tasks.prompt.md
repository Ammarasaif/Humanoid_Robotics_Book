---
id: 3
title: Generate tasks for blog removal from Docusaurus site
stage: tasks
date: 2025-12-30
surface: agent
model: Claude Sonnet 4.5
feature: remove-blog
branch: 007-remove-blog
user: Claude Code
command: /sp.tasks
labels:
  - docusaurus
  - blog
  - removal
  - tasks
  - implementation
links:
  spec: specs/007-remove-blog/spec.md
  plan: specs/007-remove-blog/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-remove-blog/tasks.md
tests: []
---

# Generate tasks for blog removal from Docusaurus site

## User Input

```
sp.tasks: Execute Docusaurus blog removal plan

Reference sp.plan output for "Remove Docusaurus blog feature completely".

Generate **5-8 atomic tasks** for sp.exec.

### Plan Recap
- Goal: Remove /blog route, navbar links, blog directory, and blog plugin.
- Key files: docusaurus.config.js, blog/*, possible sidebars.js.
- Success: Clean build/run, /blog → 404, no regressions.

### Required Tasks
1. Disable blog plugin: Set `blog: false` in preset-classic (or remove plugin) in docusaurus.config.js.
2. Remove "Blog" navbar item from docusaurus.config.js.
3. Delete blog directory: `rm -rf blog/`.
4. Remove blog sidebar references (if sidebars.js exists).
5. Verify build: `npm run build` (no blog errors).
6. Test routes: `npm start`, check /blog 404 and other pages intact.
7. Commit: `git add . && git commit -m "feat: remove blog feature"`.

### Task Format (SpecKitPlus YAML)
Each task:
```yaml
- id: TASK-001
  title: "Disable blog plugin"
  files: ["docusaurus.config.js"]
  type: modify
  changes: |
    ```diff
    -   blog,
    +   blog: false,
```

## Response Summary

Created a comprehensive tasks file (specs/007-remove-blog/tasks.md) with 17 atomic tasks organized in 6 phases:

1. **Setup Phase**: Verify current project state
2. **Foundational Phase**: Disable blog plugins in all config files (3 tasks in parallel)
3. **US1 Phase**: Remove navigation links from all config files (6 tasks across all config files)
4. **US2 Phase**: Remove blog directory and content (1 task)
5. **US3 Phase**: Verify functionality and testing (4 tasks)
6. **Polish Phase**: Commit changes and update documentation (2 tasks)

The tasks follow the required checklist format with proper IDs, story labels, and parallelization markers. The implementation strategy ensures a clean, working site with the blog completely removed while maintaining all other functionality. Dependencies and parallel execution opportunities are clearly documented.