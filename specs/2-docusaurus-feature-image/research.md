# Research: Update Docusaurus Homepage Feature Image

**Feature**: Update Docusaurus homepage feature image
**Date**: 2025-12-24
**Branch**: 2-docusaurus-feature-image

## Current State Analysis

### File: `src/components/HomepageFeatures/index.tsx`
- Uses a mixed approach with both SVG components and a separate SimpleFeatureComponent
- The first feature uses `undraw_docusaurus_mountain.svg` via a required import
- The `SimpleFeatureComponent` function renders this SVG with the `featureSvg` class
- Other features use the standard `Feature` component with different SVGs
- CSS class `featureSvg` sets height/width to 200px

### Target Image
- File: `static/img/embodied_intelligence.jpeg`
- Format: JPEG (534KB)
- Purpose: Replace the default mountain SVG with a relevant image for the Physical AI Humanoid Robotics Book

### Implementation Strategy
- Replace the `ImageIcon` component in `SimpleFeatureComponent` with an `<img>` tag
- Maintain the same `styles.featureSvg` CSS class
- Use `useBaseUrl` for proper path resolution in Docusaurus
- Preserve all other functionality and layout

## Technical Considerations

### Docusaurus Image Handling
- Docusaurus recommends using `useBaseUrl` for static assets
- Static images go in `static/img/` directory (already done)
- The `useBaseUrl` hook is already imported in the component

### CSS Compatibility
- The existing `featureSvg` class in styles.module.css will be preserved
- Dimensions will remain at 200px height/width as defined in the CSS
- Responsive behavior should remain unchanged

### Accessibility
- The SVG currently has `role="img"` attribute
- For the img tag, we should include appropriate `alt` text
- Maintain the same accessibility properties