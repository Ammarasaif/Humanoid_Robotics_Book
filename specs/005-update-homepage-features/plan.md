# Implementation Plan: Update Homepage Features for Physical AI/Humanoid Robotics Book

**Feature Spec**: `specs/005-update-homepage-features/spec.md`
**Branch**: `005-update-homepage-features`
**Created**: 2025-12-25
**Status**: Draft

## Technical Context

- **Frontend Framework**: Docusaurus v3+
- **Component Location**: `src/components/HomepageFeatures/index.tsx`
- **Styles Location**: `src/components/HomepageFeatures/styles.module.css`
- **Image Directory**: `static/img/`
- **Existing Images**:
  - `autonomous_system.jpg` (already exists)
  - `cognitive Embodiment.jpeg` (already exists)
  - `real_world_intelligence.webp` (already exists)
- **Current Implementation**: Uses both SVG and image imports in FeatureList array
- **Required Changes**: Replace titles, descriptions, and SVGs with new images for all three features

## Architecture Decision Record (ADR)

### Decision: Image Format Strategy
- **Chosen**: Use existing images with mixed formats (JPG, JPEG, WebP)
- **Rationale**: Images already exist in the repository with appropriate academic content
- **Alternatives Considered**: Standardize all to same format, convert all to PNG
- **Trade-offs**: Maintains existing assets vs. format consistency

### Decision: Component Structure Preservation
- **Chosen**: Keep existing JSX structure and only modify content elements
- **Rationale**: Maintains layout, spacing, and responsive behavior as required
- **Trade-offs**: Limited customization vs. maintaining compliance with requirements

## Constitution Check

### Phase Boundary Compliance
- ✅ This change is within Phase 2 (Custom Book UI) scope
- ✅ No content rewriting - only UI/visual updates
- ✅ No backend systems affected

### Quality & Safety Guarantees
- ✅ Changes maintain independent buildability
- ✅ No breaking changes to previous phases
- ✅ Architecture decisions documented

### Academic Rigor & Technical Accuracy
- ✅ Content aligns with Physical AI/Humanoid Robotics academic focus
- ✅ Professional, academic-level descriptions implemented

## Implementation Gates

### Gate 1: Technical Feasibility
- ✅ Component structure allows for required changes
- ✅ Images exist in appropriate formats
- ✅ Docusaurus supports required image formats

### Gate 2: Design Compliance
- ✅ Layout and spacing will match default Docusaurus UI standards
- ✅ Responsive behavior preserved
- ✅ Visual consistency maintained

### Gate 3: Build Compatibility
- ✅ TypeScript types compatible with changes
- ✅ CSS modules will maintain styling
- ✅ Import/export structure preserved

## Phase 0: Research & Unknowns Resolution

### Research Findings
- **Component Structure**: Current component uses FeatureList array with title, Svg/imageUrl, and description properties
- **Image Import Strategy**: Docusaurus supports both SVG and image imports via require() or import statements
- **CSS Styling**: Current .featureSvg class handles sizing and responsive behavior
- **Existing Implementation**: The component already has one feature using an image (realWorldIntelligence) and two using SVGs

### Decision Points Resolved
- **Image Import Method**: Use import statements for new images to match existing pattern
- **Image Format Handling**: Support mixed formats (JPG, JPEG, WebP) as Docusaurus handles these natively
- **Alt Text Strategy**: Use feature titles as alt text for accessibility

## Phase 1: Design & Contracts

### Data Model
```
FeatureItem: {
  title: string,
  imageUrl?: string,  // Used for images instead of Svg
  description: ReactNode
}
```

### Component Contract
- **Input**: None (static component)
- **Output**: Three feature cards with academic content
- **Dependencies**: React, clsx, Docusaurus Heading component, CSS modules
- **Side Effects**: Renders homepage features section

### CSS Requirements
- **Image Sizing**: max-width: 100%, height: auto for responsive behavior
- **Centering**: Images centered within feature columns
- **Responsive**: Maintain current breakpoints for mobile optimization

## Implementation Strategy

### Step 1: Update Image Imports
- Replace SVG imports with image imports for all three features
- Import images from `@site/static/img/` path

### Step 2: Update FeatureList Array
- Replace "Easy to Use" with "Autonomous Systems"
- Replace "Focus on What Matters" with "Cognitive Embodiment"
- Replace "Powered by React" with "Real-World Intelligence"
- Update all descriptions with academic content
- Replace SVG references with image references

### Step 3: Update CSS (if needed)
- Ensure .featureSvg class maintains proper sizing for new images
- Add max-width constraints if needed for image consistency

### Step 4: Testing Strategy
- Verify all images load correctly
- Check responsive behavior on different screen sizes
- Validate build process completes without errors
- Confirm accessibility (alt text, semantic structure)