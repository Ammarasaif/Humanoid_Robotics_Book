# Quickstart: Navbar Logo Replacement

## Overview

This guide explains how to replace the default Docusaurus navbar logo with a custom humanoid logo while maintaining layout and theme compatibility.

## Prerequisites

- Node.js and npm installed
- Docusaurus v3+ project running
- Custom logo file: `static/img/humanoid_logo.jpg`

## Implementation Steps

1. **Verify Logo File**
   ```bash
   # Confirm the logo file exists
   ls -la static/img/humanoid_logo.jpg
   ```

2. **Update Configuration**
   - Open `docusaurus.config.ts`
   - Locate the `themeConfig.navbar.logo` property
   - Change the `src` value from `img/logo.svg` to `img/humanoid_logo.jpg`

3. **Example Configuration**
   ```typescript
   // In docusaurus.config.ts
   themeConfig: {
     navbar: {
       logo: {
         src: 'img/humanoid_logo.jpg',  // Updated logo path
         alt: 'Physical AI & Humanoid Robotics',  // Optional alt text
       },
       // ... other navbar config
     },
     // ... other theme config
   }
   ```

4. **Test the Changes**
   ```bash
   # Start the development server
   npm run start
   ```

## Verification

- [ ] Navbar displays the humanoid logo instead of the dinosaur logo
- [ ] Logo appears correctly in both light and dark themes
- [ ] No layout shifts or console warnings occur
- [ ] Logo scales properly on mobile and desktop views
- [ ] All pages show the new logo consistently

## Troubleshooting

**Logo not displaying**: Verify the file path is correct and the image file exists in `static/img/`

**Layout issues**: Ensure the logo dimensions are appropriate (similar to original logo size)

**Theme issues**: The same logo file works for both light and dark themes in Docusaurus