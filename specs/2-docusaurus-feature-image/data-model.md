# Data Model: Update Docusaurus Homepage Feature Image

**Feature**: Update Docusaurus homepage feature image
**Date**: 2025-12-24
**Branch**: 2-docusaurus-feature-image

## Component Structure

### Current Feature Types
```typescript
type SimpleFeature = {
  title: string;
  description: ReactNode;
};

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};
```

### Component Functions
- `SimpleFeatureComponent`: Renders the first feature with SVG
- `Feature`: Renders other features with SVG components
- `HomepageFeatures`: Main component that renders all features

## Data Flow
1. `SimpleFeatureData` contains static data for the first feature
2. `FeatureList` contains array of other features with SVG imports
3. `HomepageFeatures` component renders the first feature using `SimpleFeatureComponent`
4. Other features are mapped and rendered using the `Feature` component

## Static Assets
- Source image: `static/img/embodied_intelligence.jpeg`
- CSS class: `featureSvg` (defined in styles.module.css)
- CSS properties: height: 200px, width: 200px

## Dependencies
- Docusaurus `useBaseUrl` hook for asset path resolution
- CSS modules for styling
- React for component rendering