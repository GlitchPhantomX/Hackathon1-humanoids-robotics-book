# Translation Integration Pattern Documentation

## Overview
This document describes the pattern for integrating the multi-language translation system into Docusaurus documentation pages. The approach wraps existing content with the `TranslatedContent` component to enable seamless language switching while preserving all original styling and functionality.

## Integration Pattern

### Basic Structure
```md
---
sidebar_position: 1
title: 'Page Title'
description: 'Page description'
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';
import TranslatedContent from '@site/src/components/TranslatedContent';

<TranslatedContent chapterId="chapter-id">

[ORIGINAL PAGE CONTENT HERE]

</TranslatedContent>
```

### Key Components

1. **Import Statement**: Import the `TranslatedContent` component
2. **Wrapper Element**: Use `<TranslatedContent chapterId="unique-id">` to wrap content
3. **Chapter ID**: Unique identifier matching the directory structure (e.g., "00-introduction", "01-ros2")

### Best Practices

#### 1. Preserve All Original Content
- Do not modify existing HTML tags
- Maintain all CSS classes and IDs
- Keep all custom components and imports
- Preserve code blocks and language specifications

#### 2. Maintain Page Structure
- Keep frontmatter unchanged
- Preserve all import statements
- Maintain the exact same content hierarchy
- Don't alter relative links or paths

#### 3. Component Placement
- Place `TranslatedContent` wrapper after all imports
- Ensure wrapper encompasses all page content
- Close the component at the end of the file
- Don't wrap individual sections; wrap entire page

### Example Implementation

#### Before Integration:
```md
---
sidebar_position: 1
title: 'ROS 2: The Robotic Nervous System'
description: 'Core ROS 2 concepts and architecture for humanoid robotics'
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={10} />
<h1 className="main-heading">ROS 2: The Robotic Nervous System</h1>
<p>This module introduces you to Robot Operating System 2...</p>
```

#### After Integration:
```md
---
sidebar_position: 1
title: 'ROS 2: The Robotic Nervous System'
description: 'Core ROS 2 concepts and architecture for humanoid robotics'
---

import ReadingTime from '@site/src/components/ReadingTime';
import TranslatedContent from '@site/src/components/TranslatedContent';

<TranslatedContent chapterId="01-ros2">

<ReadingTime minutes={10} />
<h1 className="main-heading">ROS 2: The Robotic Nervous System</h1>
<p>This module introduces you to Robot Operating System 2...</p>

</TranslatedContent>
```

### Content Preservation Rules

#### HTML Elements
- All tags preserved: `<h1>`, `<p>`, `<div>`, `<table>`, etc.
- All attributes maintained: `className`, `id`, `style`, etc.
- Structural hierarchy remains intact

#### CSS Classes
- All Docusaurus classes preserved: `main-heading`, `second-heading`, `underline-class`
- Custom classes maintained: `border-line`, etc.
- No class names modified or removed

#### Special Components
- Code blocks with syntax highlighting preserved
- Mermaid diagrams remain functional
- Custom Docusaurus components continue to work
- Relative links maintain correct paths

### Directory Structure Mapping

The `chapterId` in the `TranslatedContent` component corresponds to the directory structure:

- `docs/00-introduction/index.md` → `chapterId="00-introduction"`
- `docs/01-ros2/index.md` → `chapterId="01-ros2"`
- `docs/02-simulation/index.md` → `chapterId="02-simulation"`
- `docs/03-isaac/index.md` → `chapterId="03-isaac"`
- `docs/04-vla/index.md` → `chapterId="04-vla"`
- `docs/05-capstone/index.md` → `chapterId="05-capstone"`

### Translation File Structure

Translation files follow this structure in `src/translations/{language}/{chapterId}.json`:

```json
{
  "meta": {
    "title": "Translated Title",
    "description": "Translated Description",
    "language": "ur",
    "chapter": "01-ros2",
    "sourceFile": "docs/01-ros2/index.md",
    "lastUpdated": "2025-12-15T10:30:00Z"
  },
  "content": {
    "headings": {},
    "paragraphs": {},
    "lists": {}
  },
  "html": "<h1>Translated HTML Content...</h1>"
}
```

### Error Handling

- If translation is unavailable, original content is displayed
- If language is English, original content is displayed
- If authentication fails, appropriate messages are shown
- Loading states provide user feedback during translation fetch

### RTL Support

- Right-to-left languages (Urdu, Arabic) automatically apply RTL direction
- Code blocks remain left-to-right for readability
- Mermaid diagrams remain left-to-right
- All styling adjusts appropriately for RTL languages

This integration pattern ensures seamless multilingual support while maintaining the integrity of the original documentation structure and functionality.