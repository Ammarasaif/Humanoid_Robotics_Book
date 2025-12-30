# Data Model: Replace Homepage Feature Image with Custom Image

## Entity: HomepageFeature

**Description**: Represents a feature displayed in the homepage features section

**Fields**:
- `title`: string - The title of the feature (e.g., "Autonomous Systems")
- `imageUrl`: string - The path to the feature image file (e.g., "/img/image.png")
- `description`: ReactNode - The descriptive text for the feature

**Validation Rules**:
- `title` must be a non-empty string
- `imageUrl` must be a valid path to an image file in the static directory
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
- `filename`: string - name of the image file (e.g., "image.png")
- `format`: string - image format (JPEG or PNG)
- `path`: string - location in the static directory (e.g., "/static/img/image.png")
- `dimensions`: object - width and height specifications for responsive display

**Validation Rules**:
- `filename` must exist in the static directory
- `format` must be either JPEG or PNG
- `path` must be a valid static asset path
- `dimensions` should match the original SVG to maintain layout consistency