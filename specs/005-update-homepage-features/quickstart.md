# Quickstart Guide: Update Homepage Features

## Prerequisites
- Node.js and npm installed
- Docusaurus project set up
- Required images in `static/img/` directory:
  - `autonomous_system.jpg`
  - `cognitive Embodiment.jpeg`
  - `real_world_intelligence.webp`

## Setup Steps

### 1. Clone and Install Dependencies
```bash
npm install
```

### 2. Verify Image Assets
Ensure the following images exist in `static/img/`:
- `autonomous_system.jpg`
- `cognitive Embodiment.jpeg`
- `real_world_intelligence.webp`

### 3. Update Component
Modify `src/components/HomepageFeatures/index.tsx` with:
- New titles for all three features
- Academic-level descriptions
- Image imports instead of SVGs
- Proper alt text for accessibility

### 4. Run Development Server
```bash
npm run start
```

### 5. Verify Changes
- Check that all three features display correctly
- Verify images load without errors
- Confirm responsive behavior on different screen sizes
- Test that build completes without warnings

## Development Commands
```bash
# Start development server
npm run start

# Build for production
npm run build

# Check for build errors
npm run build
```

## Common Issues & Solutions
- **Image not loading**: Verify file path and name match import statement
- **Build errors**: Check TypeScript syntax and import paths
- **Responsive issues**: Verify CSS classes are applied correctly