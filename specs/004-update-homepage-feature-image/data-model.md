# Data Model: Replace Homepage Feature with Physical AI/Humanoid Robotics Feature

## Entity: HomepageFeature

**Description**: Represents a feature displayed in the homepage features section

**Fields**:
- `title`: string - The title of the feature (e.g., "Real world Intelligence")
- `Svg?`: React.ComponentType<React.ComponentProps<'svg'>> - The SVG component for the feature icon (optional)
- `imageUrl?`: string - The path to the feature image file (e.g., "/img/Real_world_intelligence.jpg") (optional)
- `description`: ReactNode - The descriptive text for the feature

**Validation Rules**:
- `title` must be a non-empty string
- `Svg` and `imageUrl` cannot both be null (one must be provided)
- `description` must be a valid ReactNode containing the required descriptive text

## Entity: FeatureList

**Description**: An array of HomepageFeature entities that make up the features section

**Fields**:
- `features`: Array<HomepageFeature> - The collection of features to display

**Constraints**:
- Must contain at least one feature
- Each feature must have a unique title for proper rendering

## Entity: CustomFeatureImage

**Description**: A static image file that replaces the default SVG in the homepage features section

**Fields**:
- `filename`: string - name of the image file (e.g., "Real_world_intelligence.jpg")
- `format`: string - image format (JPEG)
- `path`: string - location in the static directory (e.g., "/static/img/Real_world_intelligence.jpg")
- `dimensions`: object - width and height specifications for responsive display

**Validation Rules**:
- `filename` must exist in the static directory
- `format` must be JPEG
- `path` must be a valid static asset path
- `dimensions` should match the original SVG to maintain layout consistency