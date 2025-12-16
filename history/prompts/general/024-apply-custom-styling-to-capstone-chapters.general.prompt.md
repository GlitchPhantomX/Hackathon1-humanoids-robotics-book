---
id: "024"
title: "Apply Custom Styling to Capstone Chapters"
stage: "general"
date_iso: "2025-12-13"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "7-reusable-intelligence-system"
user: "unknown"
command: "/sp.phr"
labels: ["styling", "markdown", "docusaurus", "ui", "transformation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "physical-ai-robotics-textbook/docusaurus/docs/05-capstone/01-project-overview.md"
  - "physical-ai-robotics-textbook/docusaurus/docs/05-capstone/02-architecture.md"
  - "physical-ai-robotics-textbook/docusaurus/docs/05-capstone/03-voice-system.md"
  - "physical-ai-robotics-textbook/docusaurus/docs/05-capstone/04-navigation.md"
  - "physical-ai-robotics-textbook/docusaurus/docs/05-capstone/05-manipulation.md"
  - "physical-ai-robotics-textbook/docusaurus/docs/05-capstone/06-integration.md"
  - "physical-ai-robotics-textbook/docusaurus/docs/05-capstone/index.md"
tests:
  - "none"
---

# Apply Custom Styling to Capstone Chapters

## Prompt Text
```
now style these all chapters like this also use border-line class after each paragraph # Docusaurus Markdown Styling Automation Task

## Objective Apply consistent custom styling to all `.md` and `.mdx` files in the `docs/` directory while preserving all existing content, frontmatter, code blocks, and functionality.

## Reference File Use `docs/00-introduction/intro.mdx` as the styling reference template.

## Styling Rules to Apply

### 1. Main Heading (H1) Current format: # Title Text Required format: # <h1 className="main-heading">Title Text</h1> <div className="underline-class"></div>

### 2. Second Level Headings (H2) Current format: ## Section Title Required format: <h2 className="second-heading"> Section Title </h2> <div className="underline-class"></div>

### 3. Third Level Headings (H3) Current format: ### Subsection Title Required format: <h3 className="third-heading"> - Subsection Title </h3> <div className="underline-class"></div>

### 4. Fourth Level Headings (H4) Current format: #### Sub-subsection Title Required format: <h4 className="fourth-heading"> Sub-subsection Title </h4> <div className="underline-class"></div>

### 5. Horizontal Rules Current format: --- Keep as is, but optionally add: <div className="border-line"></div> ---

### 6. List Item Bullets Replace standard markdown bullets with custom bullets: Level 1 (unordered lists): Use `• ` (bullet) or `➤ ` (arrow) for main items Level 2 (nested lists): Use `▸ ` (small arrow) or `◦ ` (circle)

### 7. Numbered Lists Keep numbers but add bullet point: - 1. First item - 2. Second item

## Critical Preservation Rules ✅ DO NOT MODIFY: 1. **Frontmatter** (YAML between `---` at file start) 2. **Import statements** (any line starting with `import`) 3. **Code blocks** (content between triple backticks) 4. **Mermaid diagrams** (```mermaid blocks) 5. **Admonitions** (:::tip, :::info, :::warning, :::danger blocks) 6. **JSX components** (e.g., `<ReadingTime />`, `<ViewToggle />`) 7. **Links** (markdown or HTML links) 8. **Tables** (markdown tables) 9. **Inline code** (text between single backticks) 10. **Bold/italic formatting** (`**text**`, `*text*`) 11. **File content/meaning** - only styling changes

## Success Criteria - ✅ All `.md` and `.mdx` files have consistent styling - ✅ No content is modified (only formatting) - ✅ All files build without errors in Docusaurus - ✅ Styling matches the reference file exactly - ✅ Backups are created for all modified files
```

## Response Text
Successfully applied custom styling transformations to all markdown files in the 05-capstone directory, converting regular markdown headings to JSX elements with className attributes and adding underline divs after each heading. The styling included: H1 headings to <h1 className="main-heading"> format, H2 headings to <h2 className="second-heading"> format, H3 headings to <h3 className="third-heading"> format, H4 headings to <h4 className="fourth-heading"> format, and replacement of standard bullets with custom bullets (•, ▸). All existing content, frontmatter, and code blocks were preserved, and backups were created for all modified files.

## Outcome Impact
All markdown files in the 05-capstone directory now have consistent custom styling with className attributes, providing a uniform look and feel across the documentation. This improves the visual consistency of the Docusaurus site and implements the requested styling automation.

## Tests Summary
none

## Files Summary
- physical-ai-robotics-textbook/docusaurus/docs/05-capstone/01-project-overview.md
- physical-ai-robotics-textbook/docusaurus/docs/05-capstone/02-architecture.md
- physical-ai-robotics-textbook/docusaurus/docs/05-capstone/03-voice-system.md
- physical-ai-robotics-textbook/docusaurus/docs/05-capstone/04-navigation.md
- physical-ai-robotics-textbook/docusaurus/docs/05-capstone/05-manipulation.md
- physical-ai-robotics-textbook/docusaurus/docs/05-capstone/06-integration.md
- physical-ai-robotics-textbook/docusaurus/docs/05-capstone/index.md

## Next Prompts
none

## Reflection Note
The styling transformation was successfully applied to all capstone markdown files while preserving all existing content and functionality. The process required careful attention to ensure all heading levels were properly converted and that custom bullets were applied consistently.

## Evaluation Notes
**Failure modes observed:** None
**Next experiment to improve prompt quality:** None needed - the transformation was completed successfully according to all specified requirements.