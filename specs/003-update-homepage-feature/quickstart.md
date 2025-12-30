# Quickstart Guide: Replace Homepage Feature Image with Custom Image

**Feature**: 003-update-homepage-feature
**Date**: 2025-12-24

## Overview
This guide provides step-by-step instructions for replacing the default homepage feature image in Docusaurus v3 with a custom JPEG/PNG image while maintaining the same UI layout and responsive design.

## Prerequisites
- Node.js and npm installed
- Docusaurus v3 project set up
- Custom image file (JPEG/PNG format) ready for upload

## Steps

### 1. Add the Custom Image to Static Directory
```bash
# Place your custom image in the static directory
cp path/to/your/image.png static/img/image.png
```

### 2. Update the Homepage Features Component
1. Open `src/components/HomepageFeatures/index.tsx`
2. Locate the feature object that currently uses `undraw_docusaurus_tree.svg`
3. Update the import statement and image reference:

```ts
// Replace the import from:
// import DocusaurusSvg from '@site/static/img/undraw_docusaurus_tree.svg';

// To:
import Image from '@site/static/img/image.png';
```

4. Update the feature object to use the new image:
```ts
{
  title: 'Autonomous Systems',
  // Replace Svg: DocusaurusSvg with:
  imageUrl: require('@site/static/img/image.png').default,
  description: 'Advanced humanoid AI combining physical intelligence and autonomous decision-making.',
}
```

### 3. Verify Responsive Design
- Test the image display on different screen sizes
- Ensure the image maintains proper aspect ratio
- Verify no layout shifts occur

### 4. Build and Test
```bash
# Build the site to ensure everything works correctly
npm run build

# Serve the site locally to test
npm run serve
```

## Common Issues and Solutions

### Image not displaying
- Ensure the image file is in the correct location (`static/img/`)
- Verify the import path is correct
- Check that the image file isn't corrupted

### Layout disruption
- Ensure the new image has similar dimensions to the original
- Check CSS classes are still applied correctly
- Verify responsive design properties are maintained

### Performance issues
- Optimize the image file size before adding
- Consider using appropriate image formats for web (PNG for graphics, JPEG for photos)

## Verification Checklist
- [ ] Custom image displays correctly in the feature component
- [ ] Image maintains responsive design across screen sizes
- [ ] No layout disruptions or visual regressions
- [ ] Site builds successfully without errors
- [ ] Image loads efficiently without performance impact
- [ ] Feature title and description remain unchanged