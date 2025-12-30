# Research: Replace Homepage Feature with Physical AI/Humanoid Robotics Feature

## Decision: Image Format and Optimization

**Rationale**: Using JPEG format for the "Real_world_intelligence.jpg" image. JPEG is ideal for photographic images with many colors and gradients, which is appropriate for real-world intelligence imagery. The image should be optimized for web use while maintaining quality.

**Alternatives considered**:
- PNG: Better for images with transparency or simple graphics, but larger file size for photographic content
- WebP: Modern format with excellent compression but limited browser support in older browsers
- SVG: Vector format but not suitable for photographic content

## Decision: Image Sizing and Dimensions

**Rationale**: Maintaining similar dimensions to the original `undraw_docusaurus_mountain.svg` image to ensure no layout changes. Docusaurus feature components typically expect images around 200-300px width for optimal display in feature cards. The image should be sized appropriately before implementation to avoid performance issues.

**Alternatives considered**:
- Different dimensions: Could cause layout disruption and require CSS adjustments
- Using original SVG sizing: Maintains consistency with existing design patterns

## Decision: Implementation Approach

**Rationale**: Following Docusaurus best practices for static image handling by placing the image in the `static/img/` directory and importing it in the component. This ensures proper bundling and optimization by Docusaurus' build process. The component needs to be updated to support both SVG and image URL formats.

**Alternatives considered**:
- Inline base64 encoding: Would increase bundle size and reduce caching benefits
- External image hosting: Would add dependency on external resources and potential loading delays
- CSS background images: Would complicate responsive design and accessibility

## Decision: Responsive Design Considerations

**Rationale**: Using Docusaurus' existing responsive image handling patterns to ensure the image scales appropriately across different screen sizes. The image should maintain its aspect ratio and fit within the existing feature component container without causing layout shifts.

**Alternatives considered**:
- Fixed dimensions: Could cause display issues on mobile devices
- Different responsive strategies: Would require additional CSS overrides and potentially break theme consistency

## Decision: Component Type Modification

**Rationale**: The FeatureItem type needs to be updated to support both SVG components and image URLs. This requires making the Svg property optional and adding an imageUrl property to support both formats.

**Alternatives considered**:
- Separate components for SVG and image: Would require more extensive changes to the component structure
- Force all features to use the same format: Would require changing all existing features

## Key Findings

1. The target feature is the first item in `FeatureList` array in `src/components/HomepageFeatures/index.tsx` with title "Easy to Use"
2. The component currently uses SVG format exclusively but needs to support image URLs
3. The component follows Docusaurus v3 patterns and conventions for feature components
4. The layout uses Bootstrap grid classes (`col col--4`) which should remain unchanged
5. The CSS Modules approach is used with existing `.featureSvg` class that will be applied to the new image
6. Static images should be placed in the `static/img/` directory for proper bundling
7. The feature component needs modification to handle both SVG and image formats