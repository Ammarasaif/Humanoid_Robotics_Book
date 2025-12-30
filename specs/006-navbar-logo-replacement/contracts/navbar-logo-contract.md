# Navbar Logo Configuration Contract

## Overview

This contract defines the expected configuration change for the Docusaurus navbar logo replacement.

## Configuration Change

### Before
```typescript
themeConfig: {
  navbar: {
    logo: {
      src: 'img/logo.svg',  // Default Docusaurus logo
      alt: 'Docusaurus',
    },
    // ... other navbar config
  },
  // ... other theme config
}
```

### After
```typescript
themeConfig: {
  navbar: {
    logo: {
      src: 'img/humanoid_logo.jpg',  // Custom humanoid logo
      alt: 'Physical AI & Humanoid Robotics',  // Updated alt text (optional)
    },
    // ... other navbar config
  },
  // ... other theme config
}
```

## Expected Behavior

1. **Logo Display**: The navbar should display `humanoid_logo.jpg` instead of the default dinosaur logo
2. **Theme Compatibility**: The logo should render correctly in both light and dark themes
3. **Responsive Behavior**: The logo should scale appropriately on different screen sizes
4. **Layout Preservation**: The navbar height, alignment, and padding should remain unchanged
5. **Accessibility**: The logo should have appropriate alt text for screen readers

## Validation Criteria

- [ ] Navbar logo path updated from `img/logo.svg` to `img/humanoid_logo.jpg`
- [ ] No console errors related to logo loading
- [ ] Logo appears consistently across all pages
- [ ] No layout shifts or visual regressions
- [ ] Logo maintains appropriate sizing in navbar