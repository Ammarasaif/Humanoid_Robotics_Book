# Quickstart Guide: Replace Homepage Feature with Physical AI/Humanoid Robotics Feature

## Overview
This guide provides step-by-step instructions for replacing the default "Easy to Use" homepage feature in Docusaurus v3 with a custom Physical AI/Humanoid Robotics feature using a JPEG image while maintaining the same UI layout and responsive design.

## Prerequisites
- Node.js and npm installed
- Docusaurus v3 project set up
- Custom image file "Real_world_intelligence.jpg" in static/img/ directory

## Steps

### 1. Verify Image File
```bash
# Confirm the image file exists
ls -la static/img/Real_world_intelligence.jpg
```

### 2. Update the Homepage Features Component
1. Open `src/components/HomepageFeatures/index.tsx`
2. Update the TypeScript type to support both SVG and image URL formats:

```ts
type FeatureItem = {
  title: string;
  Svg?: React.ComponentType<React.ComponentProps<'svg'>>;
  imageUrl?: string;
  description: ReactNode;
};
```

3. Add import for the new image:
```ts
import realWorldIntelligence from '@site/static/img/Real_world_intelligence.jpg';
```

4. Update the Feature component to handle both SVG and image formats:
```ts
function Feature({title, Svg, imageUrl, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {Svg ? (
          <Svg className={styles.featureSvg} role="img" />
        ) : imageUrl ? (
          <img src={imageUrl} className={styles.featureSvg} role="img" alt={title} />
        ) : null}
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}
```

5. Update the "Easy to Use" feature object to use the new image and information:
```ts
{
  title: 'Real world Intelligence',
  imageUrl: realWorldIntelligence,
  description: (
    <>
      Advanced humanoid intelligence combining perception, reasoning, and physical action in real-world environments.
    </>
  ),
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
- Consider using appropriate image formats for web (JPG for photographic content)

## Verification Checklist
- [ ] Custom image displays correctly in the feature component
- [ ] Image maintains responsive design across screen sizes
- [ ] No layout disruptions or visual regressions
- [ ] Site builds successfully without errors
- [ ] Image loads efficiently without performance impact
- [ ] Feature title and description are updated correctly
- [ ] Other features continue to work as expected