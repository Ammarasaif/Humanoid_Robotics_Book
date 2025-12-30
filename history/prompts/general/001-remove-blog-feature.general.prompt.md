---
id: 1
title: Remove blog feature from Docusaurus site
stage: spec
date: 2025-12-30
surface: agent
model: Claude Sonnet 4.5
feature: remove-blog
branch: 007-remove-blog
user: Claude Code
command: /sp.specify
labels:
  - docusaurus
  - blog
  - removal
  - configuration
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-remove-blog/spec.md
  - specs/007-remove-blog/checklists/requirements.md
tests: []
---

# Remove blog feature from Docusaurus site

## User Input

```
You are an expert Docusaurus developer following spec-driven development (SpecKitPlus patterns). Completely remove the blog feature from an existing Docusaurus site, eliminating the /blog route, navigation links, and related components, while preserving full functionality of docs, homepage, and other features.

### Project Context
- Standard Docusaurus v2+ or v3 project using @docusaurus/preset-classic.
- Blog plugin enabled by default in docusaurus.config.js.
- Possible /blog directory with posts.
- Possible navbar/sidebar links to /blog.
- Assume standard configuration (no custom overrides).

### Requirements
1. **Disable blog plugin**: Remove or set blog: false in preset-classic options within docusaurus.config.js.
2. **Clean up files**: Delete /blog directory (posts, images, etc.) if present; remove any blog sidebar configs.
3. **Remove navigation**: Update navbar in docusaurus.config.js to eliminate any {to: '/blog', label: 'Blog'} items; prevent broken links.
4. **Routing**: Ensure /blog route is gone (defaults to 404; redirect optional).
5. **No regressions**: Keep docs, homepage, search, etc. intact; site must build/run without errors (test `npm run build` and `npm start`).
6. **Best practices**: Fully disable plugin if no custom blog usage; suggest migrating kept posts to docs (primary goal: full removal); use clean commit message e.g. "feat: remove blog feature".

### Tasks
Generate atomic changes (one file per task where possible) with exact paths and code diffs.
Include verification steps (build, route checks).

### Output Format
- Series of file modifications/deletions.
- Full updated file contents where needed.
- End with build/test instructions.

Implement safely and minimally.
```

## Response Summary

Created a comprehensive specification for removing the blog feature from a Docusaurus site. The specification includes:

- User scenarios for site visitors, content editors, and developers
- Functional requirements covering plugin disabling, directory removal, navigation updates, and route handling
- Success criteria with measurable outcomes
- Assumptions about the current state and configuration

The specification ensures that the blog feature is completely removed while preserving all other functionality (docs, homepage, search, etc.) and maintains the ability to build and run the site successfully.

Files created:
- specs/007-remove-blog/spec.md: Complete feature specification
- specs/007-remove-blog/checklists/requirements.md: Quality validation checklist