# Data Model: Update Homepage Feature Image

## Overview
Data model for the homepage feature image update. This feature primarily involves UI components rather than data entities, but we'll document the relevant structures.

## Feature Structure

### FeatureItem Interface
```typescript
type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
}
```

### FeatureList Array
- **Type**: `FeatureItem[]`
- **Purpose**: Contains all homepage feature items
- **Structure**: Array of objects with title, Svg, and description properties
- **Current Items**:
  1. "Embodied Intelligence" - to be updated with new image
  2. "Focus on What Matters" - unchanged
  3. "Powered by React" - unchanged

## Image Assets

### Current Image
- **Path**: `@site/static/img/undraw_docusaurus_mountain.svg`
- **Format**: SVG
- **Usage**: Required via `require()` function in FeatureList

### New Image
- **Path**: `@site/static/img/embedding_bodied_02.webp`
- **Format**: WebP
- **Usage**: Will replace current image in "Embodied Intelligence" feature

## Component Structure

### HomepageFeatures Component
- **File**: `src/components/HomepageFeatures/index.tsx`
- **Function**: Renders feature cards in a grid layout
- **Dependencies**:
  - FeatureList array (contains feature data)
  - styles.module.css (contains styling)
  - Various Docusaurus components (Heading, Layout, etc.)

### Feature Sub-component
- **Function**: Renders individual feature card
- **Props**: title, Svg, description from FeatureItem
- **Structure**: SVG image, title heading, and description paragraph

## UI Properties

### Preserved Elements
- Feature title: "Embodied Intelligence" (unchanged)
- Feature description: Text remains the same
- Layout structure: Grid layout with 3 columns
- Styling: CSS classes and visual appearance preserved
- Responsive behavior: Mobile/tablet/desktop layouts

### Changed Elements
- Feature image: From SVG to WebP format
- Image source: Path updated to new image file