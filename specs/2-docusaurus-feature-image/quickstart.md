# Quickstart: Update Docusaurus Homepage Feature Image

**Feature**: Update Docusaurus homepage feature image
**Date**: 2025-12-24
**Branch**: 2-docusaurus-feature-image

## Prerequisites

- Node.js and npm installed
- Docusaurus development environment set up
- Target image `static/img/embodied_intelligence.jpeg` exists in project

## Implementation Steps

### 1. Modify Component File
Edit `src/components/HomepageFeatures/index.tsx`:

1. Locate the `SimpleFeatureComponent` function (lines 65-78)
2. Replace the SVG import with an img tag
3. Maintain the `styles.featureSvg` CSS class
4. Use `useBaseUrl` for the image path
5. Add appropriate alt text

### 2. Expected Code Change
Replace this:
```typescript
function SimpleFeatureComponent({title, description}: SimpleFeature) {
  const ImageIcon = require('@site/static/img/undraw_docusaurus_mountain.svg').default;
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <ImageIcon className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}
```

With this:
```typescript
function SimpleFeatureComponent({title, description}: SimpleFeature) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img
          src={useBaseUrl('/img/embodied_intelligence.jpeg')}
          className={styles.featureSvg}
          alt="Embodied Intelligence"
        />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}
```

### 3. Verify Changes
1. Stop any running development server
2. Clear Docusaurus cache: delete `.docusaurus` directory
3. Start development server: `npm run start`
4. Visit homepage and verify the new image appears
5. Check browser console for any errors
6. Test responsiveness on different screen sizes

## Testing

- Image loads correctly without errors
- CSS class `featureSvg` is preserved
- Layout and spacing remain unchanged
- No console warnings or errors
- Works in both development and production builds