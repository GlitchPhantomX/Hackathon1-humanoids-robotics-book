# Translation Contribution Guidelines

## Overview

This document provides guidelines for contributing translations to the Physical AI & Humanoid Robotics Textbook. We welcome contributions to make this educational resource accessible in more languages.

## Getting Started

### Prerequisites
- Proficiency in both English and the target language
- Understanding of technical terminology in robotics and AI
- Familiarity with Markdown formatting
- Access to appropriate translation tools (optional but helpful)

### Repository Setup
1. Fork the repository
2. Clone your fork locally
3. Create a new branch for your translation work
4. Follow the setup instructions in the main README

## Translation Process

### 1. Choose What to Translate

#### Priority Content
1. **Introduction chapters** - Welcome, prerequisites, syllabus
2. **Core technical chapters** - ROS2, Simulation, Isaac
3. **Advanced chapters** - VLA, Capstone
4. **Supporting documentation** - Personalization, deployment guides

#### File Structure
```
docs/
├── 00-introduction/
├── 01-ros2/
├── 02-simulation/
├── 03-isaac/
├── 04-vla/
└── 05-capstone/
```

### 2. Translation Workflow

#### Step 1: Set Up Your Environment
```bash
# Navigate to the docusaurus directory
cd physical-ai-robotics-textbook/docusaurus

# Install dependencies
npm install

# Start the development server
npm run start
```

#### Step 2: Create Translation Directory
```bash
# For a new language code 'new-locale'
mkdir -p i18n/new-locale/docusaurus-plugin-content-docs/current
```

#### Step 3: Copy English Files as Templates
```bash
# Copy all English files to your new locale directory
cp -r docs/* i18n/new-locale/docusaurus-plugin-content-docs/current/
```

#### Step 4: Translate Content
- Translate only the content, not the structure
- Keep all HTML tags, class names, and component imports
- Maintain all code blocks in English
- Preserve all links and image references

### 3. File Structure and Components

#### Markdown File Structure
Each translated file should maintain this structure:

```markdown
---
sidebar_position: 1
title: "TRANSLATED TITLE"
description: "TRANSLATED DESCRIPTION"
---

import TranslateButton from '@site/src/components/TranslateButton';

<TranslateButton />

# Content Title

[Your translated content...]

```

#### Required Components
- **TranslateButton**: Must be included at the beginning of each chapter
- **Frontmatter**: Title and description must be translated
- **HTML Structure**: Preserve all class names and structure

### 4. Translation Guidelines

#### Content Translation
- **Accuracy**: Prioritize technical accuracy over literal translation
- **Consistency**: Use consistent terminology throughout
- **Cultural Sensitivity**: Adapt examples to be culturally appropriate
- **Tone**: Maintain the educational and professional tone

#### Technical Terms
- **Keep in English**: Maintain English for technical terms
- **Add Context**: Provide translations in parentheses when helpful
- **Consistency**: Use the same translation for the same term

#### Examples:
```
✅ Keep English: "We'll use ROS 2 (Robot Operating System 2) for..."
❌ Don't translate: "We'll use نظام تشغيل الروبوت 2 (Robot Operating System 2) for..."

✅ Add context: "The node (کوکا) communicates with..."
```

### 5. Preserving Structure and Functionality

#### HTML and Markdown Elements
- **Headers**: Keep `#`, `##`, `###` structure
- **Lists**: Preserve ordered and unordered list formatting
- **Code Blocks**: Keep exactly as is (in English)
- **Inline Code**: Keep as is (e.g., `command`, `variable`)
- **Links**: Keep all links functional
- **Images**: Keep image references unchanged

#### Components and Imports
```markdown
✅ Keep: import TranslateButton from '@site/src/components/TranslateButton';
✅ Keep: <TranslateButton />
✅ Keep: className="main-heading"
✅ Keep: :::tip, :::caution blocks
```

#### Styling and Classes
- **Preserve**: All `className` attributes
- **Preserve**: All HTML structure
- **Preserve**: All component tags

### 6. Quality Assurance

#### Self-Review Checklist
- [ ] Translation is accurate and natural-sounding
- [ ] Technical terms are handled appropriately
- [ ] All structure and formatting is preserved
- [ ] Code blocks remain in English
- [ ] Links and references work correctly
- [ ] The page renders correctly in the browser

#### Testing Your Translation
```bash
# Test the specific locale
npm run start -- --locale new-locale

# Verify all pages render correctly
# Check navigation and functionality
# Verify the TranslateButton works
```

## Style Guide

### Writing Style
- **Clear and Concise**: Use simple, clear language
- **Consistent Tone**: Maintain professional, educational tone
- **Active Voice**: Prefer active voice over passive
- **Technical Accuracy**: Prioritize accuracy over elegance

### Terminology Consistency
Create a terminology reference for your language:

| English | Target Language | Notes |
|---------|----------------|-------|
| Robot Operating System | (Keep as ROS) | Use English acronym |
| Node | (translate appropriately) | Consistent throughout |
| Topic | (translate appropriately) | Consistent throughout |

### Formatting Standards
- **Headings**: Maintain hierarchy (#, ##, ###)
- **Lists**: Keep ordered/unordered distinction
- **Emphasis**: Preserve bold and italic formatting
- **Code**: Keep all code formatting intact

## File Organization

### Directory Structure
```
i18n/
└── [locale]/
    ├── code.json                    # UI translations
    ├── docusaurus-theme-classic/    # Theme translations
    │   ├── footer.json
    │   └── navbar.json
    └── docusaurus-plugin-content-docs/
        └── current/                 # Content translations
            ├── 00-introduction/
            ├── 01-ros2/
            ├── 02-simulation/
            ├── 03-isaac/
            ├── 04-vla/
            └── 05-capstone/
```

### Naming Conventions
- **Files**: Use the same names as English files
- **IDs**: Keep all HTML IDs unchanged
- **Classes**: Preserve all CSS class names

## Submission Process

### 1. Prepare Your Changes
```bash
# Verify your translations work
npm run start -- --locale your-locale

# Test build
npm run build -- --locale your-locale
```

### 2. Create Pull Request
- **Title**: "feat: Add [Language] translation for [specific chapters]"
- **Description**: Include:
  - Which chapters/sections are translated
  - Any special considerations
  - Translation accuracy verification status

### 3. Review Process
- Translation accuracy review
- Technical accuracy verification
- Formatting and structure check
- Functionality testing

## Tools and Resources

### Recommended Tools
- **Translation Memory**: Tools like Weblate or Transifex (for large projects)
- **Text Editors**: VS Code with Markdown extensions
- **Browser Testing**: Chrome, Firefox, Safari, Edge

### Reference Materials
- **Technical Glossaries**: Official documentation for technical terms
- **Style Guides**: Language-specific technical writing guidelines
- **Cultural Adaptation**: Guidelines for cultural appropriateness

## Best Practices

### For New Contributors
1. **Start Small**: Begin with a single chapter or section
2. **Test Often**: Verify your changes frequently
3. **Ask Questions**: Reach out if you're unsure about terminology
4. **Be Consistent**: Keep terminology consistent across files

### For Reviewers
1. **Technical Accuracy**: Verify technical terms are handled correctly
2. **Cultural Appropriateness**: Ensure examples and tone are appropriate
3. **Consistency**: Check for consistent terminology and style
4. **Functionality**: Verify all components work correctly

## Common Pitfalls to Avoid

### Structure Issues
- ❌ Changing HTML class names
- ❌ Removing component imports
- ❌ Breaking code block formatting
- ❌ Modifying internal links

### Translation Issues
- ❌ Translating technical terms unnecessarily
- ❌ Changing the educational tone
- ❌ Inconsistent terminology
- ❌ Cultural insensitivity

## Maintaining Quality

### Regular Updates
- **New Content**: Translate new English content promptly
- **Feedback Integration**: Incorporate user feedback
- **Quality Improvements**: Refine translations based on usage

### Community Involvement
- **Native Speaker Review**: Engage native speakers for review
- **Technical Review**: Ensure technical accuracy
- **User Testing**: Gather feedback from actual users

## Getting Help

### Contact Information
- **GitHub Issues**: For technical questions about translation process
- **Documentation**: Check the main documentation first
- **Community**: Reach out to other translation contributors

### Resources
- **Main Documentation**: Refer to the developer guide for technical details
- **Style Guides**: Language-specific technical writing guidelines
- **Terminology**: Technical term references for your language

## Conclusion

Thank you for contributing to making the Physical AI & Humanoid Robotics Textbook accessible in multiple languages. Your efforts help make this educational resource available to a global audience. Remember to maintain technical accuracy, preserve functionality, and create natural-sounding translations that serve the educational purpose of the textbook.

Happy translating!