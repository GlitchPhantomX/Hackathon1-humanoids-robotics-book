# Translation File Schema Documentation

## Overview
This document describes the structure and validation rules for translation files used in the multi-language translation system.

## Schema Structure

### Meta Information
- `title`: String - Title of the translated content
- `description`: String - Brief description of the translated content
- `language`: Enum ('ur' | 'ar') - Target language code
- `chapter`: String - Chapter identifier (e.g., '00-introduction', '01-ros2')
- `sourceFile`: String - Original source file path for reference
- `lastUpdated`: String (ISO 8601 date-time) - Timestamp of last update

### Content Structure
- `headings`: Object mapping heading IDs to translated text
- `paragraphs`: Object mapping paragraph IDs to translated text
- `lists`: Object mapping list IDs to arrays of translated list items

### HTML Content
- `html`: String containing the complete HTML with all translations applied, preserving original structure including all tags, classes, and IDs.

## Example

```json
{
  "meta": {
    "title": "فزیکل اے آئی کا تعارف",
    "description": "Introduction to Physical AI concepts",
    "language": "ur",
    "chapter": "00-introduction",
    "sourceFile": "docs/00-introduction/index.md",
    "lastUpdated": "2025-12-15T10:30:00Z"
  },
  "content": {
    "headings": {
      "introduction": " fizikī ā'ī kā ta'āruf",
      "overview": " جائزہ "
    },
    "paragraphs": {
      "para-1": " fizikī ā'ī ek jadīd saiyans hai...",
      "para-2": " yah usūl zarūrī hai..."
    },
    "lists": {
      "key-points": [
        "pahla nuqta",
        "dosra nuqta",
        "teesra nuqta"
      ]
    }
  },
  "html": "<h1 id='introduction'> fizikī ā'ī kā ta'āruf</h1>..."
}
```

## Validation Rules

1. All meta fields are required
2. Language must be either 'ur' or 'ar'
3. Content object must include headings, paragraphs, and lists
4. HTML field must contain valid HTML with preserved structure
5. All original CSS classes and IDs must be preserved
6. Links and image paths must remain functional