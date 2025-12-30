# Data Model: Navbar Logo Configuration

## Entity: Navbar Logo

**Description**: The visual branding element displayed in the top-left of the navigation bar, representing the Physical AI/Humanoid Robotics project

**Properties**:
- `src`: String - Path to the logo image file (relative to static directory)
- `alt`: String - Alternative text for accessibility (optional, defaults to site title)
- `width`: Number - Width of the logo in pixels (optional, Docusaurus handles automatically)
- `height`: Number - Height of the logo in pixels (optional, Docusaurus handles automatically)

**Validation Rules**:
- `src` must point to a valid image file in the static directory
- `src` must use the format `img/filename.ext` for static images
- Image file must exist at the specified path
- Image format should be JPG, PNG, or SVG for best compatibility

## Entity: Theme Configuration

**Description**: The Docusaurus configuration object that controls navbar appearance and behavior

**Properties**:
- `navbar`: Object - Contains navbar-specific configuration
  - `logo`: Object - Contains logo-specific configuration
    - `src`: String - Path to the logo image file
    - `alt`: String - Alternative text for accessibility
    - `target`: String - Link target when logo is clicked (optional)

**Relationships**:
- Navbar Logo is contained within Theme Configuration
- Theme Configuration affects all pages in the Docusaurus site
- Navbar Logo is displayed consistently across all pages

## State Transitions

1. **Before Implementation**: Navbar displays default Docusaurus dinosaur logo (`img/logo.svg`)
2. **After Implementation**: Navbar displays custom humanoid logo (`img/humanoid_logo.jpg`)
3. **Theme Adaptation**: Logo appearance adapts automatically to light/dark themes
4. **Responsive Behavior**: Logo scales appropriately on different screen sizes