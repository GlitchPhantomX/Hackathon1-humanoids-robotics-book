# Data Model: Chapter-Level AI Personalization System

## Request/Response Models

### PersonalizeChapterRequest
```python
{
  "chapter_id": "string",           # Unique identifier for the chapter
  "chapter_content": "string",      # Raw markdown content of the chapter/section
  "user_profile": {
    "softwareBackground": "string", # User's software background
    "hardwareBackground": "string", # User's hardware background
    "programmingLanguages": "string", # Programming languages known
    "roboticsExperience": "string", # Robotics experience level
    "aiMlExperience": "string",     # AI/ML experience level
    "hasRosExperience": "boolean",  # Whether user has ROS experience
    "hasGpuAccess": "boolean",      # Whether user has GPU access
    "learningGoals": "string"       # User's learning goals
  }
}
```

### PersonalizeChapterResponse
```python
{
  "personalized_content": "string", # AI-transformed markdown content
  "processing_time_ms": "integer",  # Time taken for personalization
  "section_id": "string"            # ID of the section that was processed (for per-section processing)
}
```

## Frontend State Model

### PersonalizationState
```typescript
{
  "isPersonalized": "boolean",      // Whether content is currently personalized
  "isLoading": "boolean",           // Whether personalization is in progress
  "hasError": "boolean",            // Whether an error occurred
  "originalContent": "string",      // Backup of original content
  "personalizedContent": "string"   // Personalized content (when available)
}
```

## User Profile Mapping

The system maps the following Better Auth user fields to personalization parameters:

| Auth Field | Personalization Parameter | Type |
|------------|---------------------------|------|
| softwareBackground | user_profile.softwareBackground | string |
| hardwareBackground | user_profile.hardwareBackground | string |
| programmingLanguages | user_profile.programmingLanguages | string |
| roboticsExperience | user_profile.roboticsExperience | string |
| aiMlExperience | user_profile.aiMlExperience | string |
| hasRosExperience | user_profile.hasRosExperience | boolean |
| hasGpuAccess | user_profile.hasGpuAccess | boolean |
| learningGoals | user_profile.learningGoals | string |

## Content Processing Model

### Section Processing
- **Input**: Chapter content split into sections based on markdown headers
- **Processing**: Each section is sent individually to the AI service
- **Output**: Personalized sections are reassembled maintaining original structure
- **Identifier**: Each section has a unique ID for tracking and reassembly

## Error Handling Model

### API Error Response
```python
{
  "error": "string",                # Error message
  "fallback_content": "string",     # Original content as fallback
  "code": "string"                  # Error code for client handling
}
```