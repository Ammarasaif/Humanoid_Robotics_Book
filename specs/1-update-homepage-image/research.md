# Research: Update Homepage Feature Image

## Overview
Research for updating the homepage feature image in the "Embodied Intelligence" section with "embedding_bodied_02" from static/img.

## Current Implementation Analysis

### Location of Homepage Features
- **File**: `src/components/HomepageFeatures/index.tsx`
- **Structure**: Contains a FeatureList array with feature objects
- **Current "Embodied Intelligence" feature**: Located in the first position of the FeatureList array

### Image Requirements
- **Current image**: Uses SVG from `@site/static/img/undraw_docusaurus_mountain.svg`
- **New image**: `embedding_bodied_02.webp` (or other format) in `static/img/`
- **Dimensions**: Need to ensure exact match with original image to prevent layout shifts

### Image Format Investigation
- **Available formats**: The image exists as `embedding_bodied_02.webp` in `static/img/`
- **Alternative formats**: Check for .png, .jpg, or .svg versions if needed
- **Dimensions**: Need to determine original image dimensions for size matching

## Technical Approach

### Image Replacement Method
- **Current approach**: Uses `require('@site/static/img/undraw_docusaurus_mountain.svg').default` for SVG
- **New approach**: Will use `require('@site/static/img/embedding_bodied_02.webp').default` for the new image
- **Alternative**: Direct import statement if required

### Dimension Matching Strategy
- **Option 1**: Use CSS to set exact dimensions matching original
- **Option 2**: Pre-process image to match exact dimensions of original
- **Option 3**: Use Docusaurus image sizing utilities

## Decision

### Chosen Approach
- Replace the SVG image in the "Embodied Intelligence" feature with the new "embedding_bodied_02" image
- Maintain the same require() import pattern for consistency
- Use CSS classes from existing styles to maintain sizing

### Rationale
- Maintains consistency with existing code patterns
- Preserves all existing functionality and layout
- Simple implementation with minimal risk of breaking changes

### Alternatives Considered
1. **Complete component rewrite**: Would be overkill for a simple image update
2. **Different image format**: Sticking with available format to avoid additional processing
3. **Different import method**: Keeping require() pattern to match existing codebase

## Implementation Notes

### Files to Modify
- `src/components/HomepageFeatures/index.tsx` - Update the image in the FeatureList

### Files to Verify
- `src/components/HomepageFeatures/styles.module.css` - Check for any size-related styles
- `static/img/embedding_bodied_02.webp` - Verify image exists and is accessible