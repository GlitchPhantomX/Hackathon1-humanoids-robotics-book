# Hackathon Constitution: Multi-Language Translation System

## Project Overview
Adding multi-language translation support (Urdu + one professional language) to the Physical AI & Humanoid Robotics textbook while maintaining exact styling, structure, and all existing functionality.

## Core Requirements

### 1. Language Support
- **Primary Languages**: English (existing), Urdu, Hindi
- **Translation Scope**: All chapter content in `/docs` folder
- **Styling Preservation**: Exact same orange/white color scheme and component structure
- **File Structure**: Mirror English docs structure for each language

### 2. UI Components Required

#### Language Dropdown (Navbar)
```typescript
Location: src/components/Navbar or docusaurus.config.mjs
Position: Left of Login/Signup buttons with spacing
Options:
  - English (EN) 🇬🇧
  - اردو (UR) 🇵🇰
  - हिंदी (HI) 🇮🇳

Behavior:
  - Persists selection across page navigation
  - Changes entire UI (sidebar, navbar, content)
  - Maintains user position in chapter structure
```

#### Translation Toggle Button
```typescript
Location: Start of each chapter (after title, before content)
Component: <TranslateButton />
Style: Orange gradient matching PersonalizeButton
Function: Switch between EN/UR/HI for current page
```

### 3. Folder Structure

```
docusaurus/
├── docs/                          # English (existing - DO NOT MODIFY)
│   ├── 00-introduction/
│   ├── 01-ros2/
│   ├── 02-simulation/
│   └── ...
├── i18n/
│   ├── ur/                        # NEW: Urdu translations
│   │   └── docusaurus-plugin-content-docs/
│   │       └── current/
│   │           ├── 00-introduction/
│   │           │   ├── 01-welcome.md
│   │           │   ├── 02-prerequisites.md
│   │           │   └── ...
│   │           ├── 01-ros2/
│   │           └── ...
│   └── hi/                        # NEW: Hindi translations
│       └── docusaurus-plugin-content-docs/
│           └── current/
│               ├── 00-introduction/
│               └── ...
└── src/
    └── components/
        └── TranslateButton.tsx    # NEW COMPONENT
```

### 4. Translation Requirements

#### Content Translation Rules
```markdown
Source: docs/00-introduction/01-welcome.md
Target: i18n/ur/docusaurus-plugin-content-docs/current/00-introduction/01-welcome.md

MUST PRESERVE:
✓ All frontmatter (sidebar_position, title, description)
✓ All import statements (ReadingTime, PersonalizeButton)
✓ All component usage (<PersonalizeButton/>, <ReadingTime/>)
✓ All className attributes (main-heading, underline-class, border-line, etc.)
✓ All HTML structure (<div>, <h2>, etc.)
✓ All code blocks with syntax highlighting
✓ All links and navigation
✓ All exercise boxes (:::tip, :::caution)

MUST TRANSLATE:
✓ All text content
✓ Headings and subheadings
✓ Exercise descriptions
✓ Table content
✓ Navigation labels
```

#### Styling Consistency
```css
Colors (MUST MATCH):
  - Primary: Orange (#FF6B35 or current orange)
  - Secondary: White (#FFFFFF)
  - Backgrounds: Current theme colors
  - Gradients: Current orange gradients

Components (MUST REUSE):
  - .main-heading
  - .second-heading
  - .third-heading
  - .underline-class
  - .border-line
  - .full-content
  - .summary-content
  - All existing CSS classes
```

### 5. Docusaurus Configuration

```javascript
// docusaurus.config.mjs
export default {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur', 'hi'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',  // Right-to-left
        htmlLang: 'ur-PK',
      },
      hi: {
        label: 'हिंदी',
        direction: 'ltr',
        htmlLang: 'hi-IN',
      },
    },
  },

  navbar: {
    items: [
      // ... existing items
      {
        type: 'localeDropdown',
        position: 'right',
        dropdownItemsAfter: [],
        className: 'language-dropdown',
      },
      // Login/Signup buttons (with left spacing)
    ],
  },
};
```

### 6. Component Implementation

#### TranslateButton Component
```typescript
// src/components/TranslateButton.tsx
import React from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function TranslateButton() {
  const {i18n} = useDocusaurusContext();
  const history = useHistory();
  const location = useLocation();

  const switchLanguage = (locale: string) => {
    const currentPath = location.pathname;
    const newPath = currentPath.replace(/^\/(en|ur|hi)/, `/${locale}`);
    history.push(newPath);
  };

  return (
    <div className="translate-button-container">
      <button
        className="translate-button"
        onClick={() => switchLanguage('ur')}
      >
        اردو میں پڑھیں
      </button>
      <button
        className="translate-button"
        onClick={() => switchLanguage('hi')}
      >
        हिंदी में पढ़ें
      </button>
    </div>
  );
}
```

### 7. Critical Rules (DO NOT VIOLATE)

```yaml
PROTECTED FILES (DO NOT MODIFY):
  - docs/** (all English content)
  - auth-backend/** (authentication system)
  - rag-chatbot/** (existing chatbot)
  - src/components/Chatbot.tsx
  - src/components/PersonalizeButton.tsx
  - All existing styling files

MUST PRESERVE:
  - All authentication flows
  - All existing routes
  - All existing components
  - All database schemas
  - All environment variables
  - Build configuration

NEW FILES ONLY:
  - i18n/ur/** (Urdu translations)
  - i18n/hi/** (Hindi translations)
  - src/components/TranslateButton.tsx
  - Updated docusaurus.config.mjs (i18n section only)
  - Updated navbar styling for dropdown spacing
```

### 8. Implementation Steps

```bash
# Step 1: Install dependencies
npm install @docusaurus/plugin-content-docs

# Step 2: Create folder structure
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
mkdir -p i18n/hi/docusaurus-plugin-content-docs/current

# Step 3: Copy and translate content
# For each file in docs/**, create translated version in i18n/{locale}/**/

# Step 4: Update docusaurus.config.mjs
# Add i18n configuration

# Step 5: Create TranslateButton component
# Add to each chapter template

# Step 6: Update navbar
# Add language dropdown with spacing

# Step 7: Test
npm run start
# Verify all languages load correctly
# Verify styling is preserved
# Verify navigation works
# Verify RTL support for Urdu
```

### 9. Translation Guidelines

#### Chapter Structure (Example)
```markdown
# English: docs/00-introduction/01-welcome.md
---
sidebar_position: 1
title: "Welcome to Physical AI & Humanoid Robotics"
---

# Urdu: i18n/ur/docusaurus-plugin-content-docs/current/00-introduction/01-welcome.md
---
sidebar_position: 1
title: "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس میں خوش آمدید"
---

# Hindi: i18n/hi/docusaurus-plugin-content-docs/current/00-introduction/01-welcome.md
---
sidebar_position: 1
title: "फिजिकल एआई और ह्यूमनॉइड रोबोटिक्स में आपका स्वागत है"
---
```

#### Technical Terms
```yaml
Keep in English (with translation in parentheses):
  - ROS 2 (ROS 2)
  - NVIDIA Isaac (NVIDIA Isaac)
  - URDF (URDF - یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ)
  - Python (Python)
  - Node (Node - نوڈ)
  - Topic (Topic - ٹاپک)
```

### 10. Testing Checklist

```markdown
- [ ] All English content unchanged
- [ ] Urdu content displays with RTL layout
- [ ] Hindi content displays with LTR layout
- [ ] Language dropdown works in navbar
- [ ] TranslateButton appears in all chapters
- [ ] Sidebar translates with language
- [ ] Orange/white color scheme consistent
- [ ] All components render correctly
- [ ] Navigation preserves chapter position
- [ ] Authentication still works
- [ ] Chatbot still works
- [ ] Build succeeds without errors
- [ ] All styling classes preserved
```

### 11. Bonus Points Achievement

```yaml
Requirements for 50 bonus points:
  ✓ Logged user can translate content
  ✓ Urdu translation available
  ✓ Button at start of each chapter
  ✓ Content quality maintained

Extra credit opportunities:
  + Additional language (Hindi)
  + Professional UI/UX
  + Seamless language switching
  + RTL support for Urdu
```

### 12. File Priority Map

```yaml
HIGH PRIORITY (Translate First):
  - docs/00-introduction/*.md (7 files)
  - docs/01-ros2/*.md (8 files)

MEDIUM PRIORITY:
  - docs/02-simulation/*.md
  - docs/03-isaac/*.md

LOW PRIORITY:
  - docs/04-vla/*.md
  - docs/05-capstone/*.md

SUPPORTING FILES:
  - index.md files in each section
  - _category_.json files (translate labels)
```

---

## Final Notes for Claude Code

```
READ THIS CAREFULLY:
1. Use speckit to analyze existing structure
2. Preserve ALL existing files
3. Create ONLY new translation files
4. Test incrementally (one la be careful do not remove and delete any other constitution file just create a new one file named translation-system-consitution.md and create this constitution in it
```