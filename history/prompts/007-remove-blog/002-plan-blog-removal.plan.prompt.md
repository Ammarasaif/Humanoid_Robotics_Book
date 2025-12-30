---
id: 2
title: Plan removal of blog feature from Docusaurus site
stage: plan
date: 2025-12-30
surface: agent
model: Claude Sonnet 4.5
feature: remove-blog
branch: 007-remove-blog
user: Claude Code
command: /sp.plan
labels:
  - docusaurus
  - blog
  - removal
  - configuration
  - planning
links:
  spec: specs/007-remove-blog/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-remove-blog/plan.md
  - specs/007-remove-blog/tasks.md
  - specs/007-remove-blog/checklists/implementation.md
tests: []
---

# Plan removal of blog feature from Docusaurus site

## User Input

```
Remove Docusaurus blog feature completely

Goal: Fully remove the default blog feature from a standard Docusaurus v2+ or v3 site (classic preset), including the /blog route, navbar links, sidebar references, and all blog content, while ensuring no impact on docs, homepage, or other features.

Context:
- Project uses @docusaurus/preset-classic with default blog enabled.
- Blog posts may exist in /blog directory.
- Navbar likely contains a "Blog" link.
- Standard configuration assumed.

Success Criteria:
- /blog route returns 404 (or is absent).
- No "Blog" item in navbar or sidebar.
- /blog directory deleted (or empty).
- Blog plugin fully disabled in config.
- Site builds cleanly (`npm run build`) and runs (`npm start`) with no errors or warnings related to blog.
- All other features (docs, search, homepage) remain fully functional.

Constraints:
- Minimal, safe changes only.
- No custom overrides assumed unless standard.
- Prefer disabling via preset-classic blog: false over separate plugin removal.
- If blog posts exist, primary action is deletion; optionally note migration to docs.

Deliverables from sp.plan:
- List of affected files (e.g., docusaurus.config.js, /blog/*).
- High-level change summary.
- Risk assessment (e.g., potential broken links).
- Atomic task breakdown suitable for sp.exec.

Proceed with planning.
```

## Response Summary

Created a comprehensive implementation plan for removing the blog feature from the Docusaurus site:

- **plan.md**: Detailed plan outlining affected files, change summary, implementation tasks, risk assessment, and verification steps
- **tasks.md**: Atomic task breakdown with before/after examples for each configuration file modification
- **checklists/implementation.md**: Implementation checklist to track completion of all tasks

The plan identifies 4 configuration files that need updates (docusaurus.config.ts, docusaurus.config.js, docusaurus.config.ts.bak) and the blog directory with its contents that need deletion. It includes comprehensive verification steps to ensure the site continues to function properly after the blog removal.