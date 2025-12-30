# Quickstart: Update Homepage Feature Image

## Overview
Quickstart guide for implementing the homepage feature image update for the "Embodied Intelligence" section.

## Prerequisites
- Node.js (v18+)
- Docusaurus CLI installed
- Access to the `embedding_bodied_02.webp` image file in `static/img/`

## Setup
1. Clone the repository
2. Install dependencies: `npm install`
3. Verify the image exists at `static/img/embedding_bodied_02.webp`

## Development
1. Start development server: `npm run start`
2. Navigate to `src/components/HomepageFeatures/index.tsx`
3. Locate the "Embodied Intelligence" feature in the FeatureList array
4. Update the image import from the old image to the new image
5. Verify the image displays correctly without layout shifts

## Implementation Steps

### Step 1: Update Image Import
- In `src/components/HomepageFeatures/index.tsx`, find the first item in FeatureList
- Change `require('@site/static/img/undraw_docusaurus_mountain.svg').default`
- To `require('@site/static/img/embedding_bodied_02.webp').default`

### Step 2: Verify Dimensions
- Check that the new image displays with the same dimensions as the original
- If needed, adjust CSS in `styles.module.css` to maintain consistent sizing

### Step 3: Test Responsive Behavior
- Test on different screen sizes to ensure the image scales properly
- Verify the layout remains consistent with other feature cards

## Testing
1. Run development server: `npm run start`
2. Navigate to homepage and verify the new image displays correctly
3. Check that all text and layout elements remain unchanged
4. Test on different screen sizes (mobile, tablet, desktop)
5. Verify no visual regressions in other feature sections

## Build & Deployment
1. Run build command: `npm run build`
2. Verify build completes without errors
3. Test build locally: `npm run serve`
4. Deploy to production environment

## Troubleshooting
- If image doesn't load: Verify file path and name match exactly
- If layout shifts occur: Check CSS classes and dimensions
- If build fails: Verify image format is supported by Docusaurus