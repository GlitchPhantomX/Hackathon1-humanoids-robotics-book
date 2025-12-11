# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `005-robotics-textbook`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Specification for Physical AI & Humanoid Robotics Textbook"

## User Scenarios & Testing *(mandatory)*
now create specification in thi path C:\new\specs create new folder in this specs directory by using
meaningfull name # Specification for Physical AI & Humanoid Robotics Textbook

## Project Overview
Create a comprehensive, professional textbook using Docusaurus that teaches Physical AI & Humanoid Robotics. The book follows a
6-module structure with interactive features including reading time indicators and dual-view (Full Lesson/Summary) functionality.

---

## 1. Project Structure

### 1.1 Directory Organization

docusaurus/
├── docs/
│   ├── index.md
│   ├── _category_.json
│   ├── 00-introduction/
│   │   ├── _category_.json
│   │   ├── 01-welcome.md
│   │   ├── 02-prerequisites.md
│   │   ├── 03-hardware-requirements.md
│   │   ├── 04-how-to-use.md
│   │   └── 05-syllabus.md
│   ├── 01-ros2/
│   │   ├── _category_.json
│   │   ├── index.md
│   │   ├── 01-architecture.md
│   │   ├── 02-nodes-topics.md
│   │   ├── 03-services-actions.md
│   │   ├── 04-python-packages.md
│   │   ├── 05-urdf-humanoids.md
│   │   └── 06-launch-files.md
│   ├── 02-simulation/
│   │   ├── _category_.json
│   │   ├── index.md
│   │   ├── 01-gazebo-intro.md
│   │   ├── 02-urdf-sdf.md
│   │   ├── 03-sensors-plugins.md
│   │   ├── 04-world-building.md
│   │   ├── 05-ros2-integration.md
│   │   └── 06-advanced-simulation.md
│   ├── 03-isaac/
│   │   ├── _category_.json
│   │   ├── index.md
│   │   ├── 01-isaac-sim.md
│   │   ├── 02-isaac-ros.md
│   │   ├── 03-vslam-navigation.md
│   │   ├── 04-perception.md
│   │   └── 05-sim-to-real.md
│   ├── 04-vla/
│   │   ├── _category_.json
│   │   ├── index.md
│   │   ├── 01-voice-to-action.md
│   │   ├── 02-llm-planning.md
│   │   ├── 03-natural-language.md
│   │   └── 04-multimodal.md
│   └── 05-capstone/
│       ├── _category_.json
│       ├── index.md
│       ├── 01-project-overview.md
│       ├── 02-architecture.md
│       ├── 03-voice-system.md
│       ├── 04-navigation.md
│       ├── 05-manipulation.md
│       └── 06-integration.md
├── src/
│   └── components/
│       ├── ReadingTime.js
│       └── ViewToggle.js
├── static/
│   ├── css/
│   │   └── custom.css
│   └── img/
└── docusaurus.config.js


---

## 2. Component Specifications

### 2.1 ReadingTime Component

*File*: src/components/ReadingTime.js

*Purpose*: Display estimated reading time at the top of each chapter

*Props*:
- minutes (number, required): Estimated reading time in minutes

*Implementation Requirements*:
javascript
import React from 'react';

export default function ReadingTime({ minutes }) {
  return (
    <div className="reading-time-badge">
      <span className="reading-time-icon">⏱</span>
      <span className="reading-time-text">{minutes} min read</span>
    </div>
  );
}


*CSS Styling* (in static/css/custom.css):
css
.reading-time-badge {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border-radius: 20px;
  font-size: 14px;
  font-weight: 600;
  margin-bottom: 20px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.reading-time-icon {
  font-size: 18px;
}


---

### 2.2 ViewToggle Component

*File*: src/components/ViewToggle.js

*Purpose*: Toggle between Full Lesson and Summary views

*Implementation Requirements*:
javascript
import React, { useState } from 'react';

export default function ViewToggle() {
  const [view, setView] = useState('full');

  const handleToggle = (newView) => {
    setView(newView);
    document.body.setAttribute('data-view', newView);
  };

  return (
    <div className="page-header">
      <div className="header-controls">
        <button
          className={`view-toggle ${view === 'full' ? 'active' : ''}`}
          onClick={() => handleToggle('full')}
        >
          📖 Full Lesson
        </button>
        <button
          className={`view-toggle ${view === 'summary' ? 'active' : ''}`}
          onClick={() => handleToggle('summary')}
        >
          📝 Summary
        </button>
      </div>
    </div>
  );
}


*CSS Styling*:
css
.page-header {
  margin-bottom: 30px;
  border-bottom: 2px solid #e0e0e0;
  padding-bottom: 15px;
}

.header-controls {
  display: flex;
  gap: 12px;
}

.view-toggle {
  padding: 10px 20px;
  border: 2px solid #667eea;
  background: white;
  color: #667eea;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
}

.view-toggle:hover {
  background: #f0f4ff;
}

.view-toggle.active {
  background: #667eea;
  color: white;
}

/* Hide summary content by default */
.summary-content {
  display: none;
}

/* Show summary when summary view is active */
body[data-view="summary"] .full-content {
  display: none;
}

body[data-view="summary"] .summary-content {
  display: block;
}


---

## 3. Chapter Template Specification

### 3.1 Standard Chapter Structure

Every chapter MUST follow this exact template:

markdown
---
sidebar_position: [number]
title: "[Chapter Title]"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={[calculated_minutes]} />
<ViewToggle />

# [Chapter Title]

<div className="full-content">

**Module**: [Module Number - Module Name]
**Learning Objectives**:
- [Objective 1]
- [Objective 2]
- [Objective 3]

**Prerequisites**: [List prerequisites]
**Estimated Time**: [X-Y hours]

---

## Introduction

[Introduction content explaining why this chapter matters, real-world applications, and what students will build]

---

## Section 1: [Section Title]

[Detailed content with code examples, explanations, diagrams]

### Subsection 1.1: [Subsection Title]

[Content]

python title="example.py"
# Code example with comments
def example_function():
    pass


---

## Section 2: [Section Title]

[Continue with more sections...]

---

## Hands-On Exercises

:::tip Exercise [X.Y]: [Exercise Name]

**Objective**: [What to build/achieve]

**Requirements**:
1. [Requirement 1]
2. [Requirement 2]
3. [Requirement 3]

**Deliverable**: [What to submit/demonstrate]

**Test**:
bash
# Commands to verify completion
ros2 topic list


**Challenge**: [Optional advanced extension]
:::

---

## Common Issues and Debugging

:::caution Common Problems

**Problem 1: [Issue Description]**

Symptoms:
- [What you see]
- [Error messages]

Solution:
bash
# Fix commands


**Problem 2: [Issue Description]**

[Solution]
:::

---

## Summary

In this chapter, you learned:

- ✅ [Key concept 1]
- ✅ [Key concept 2]
- ✅ [Key concept 3]
- ✅ [Key concept 4]

**Key Takeaways**:
- [Critical insight 1]
- [Critical insight 2]
- [Critical insight 3]

---

## Additional Resources

**Official Documentation**:
- [Resource 1](URL)
- [Resource 2](URL)

**Tutorials**:
- [Tutorial 1](URL)

**Example Code**:
- [GitHub Repository](URL)

---

**Next**: [Next Chapter Title](./next-chapter.md)

**Navigation**: [← Previous Chapter](./previous.md) | [Next Chapter →](./next.md)

</div>

<div className="summary-content">

## 📝 Chapter Summary

### Key Concepts
- **[Concept 1]**: [Brief explanation in 1-2 sentences]
- **[Concept 2]**: [Brief explanation in 1-2 sentences]
- **[Concept 3]**: [Brief explanation in 1-2 sentences]

### Essential Code Pattern
python
# Most important code snippet from chapter
# Simplified version showing core concept


### Quick Reference
| Topic | Key Points |
|-------|------------|
| [Topic 1] | [Quick bullet points] |
| [Topic 2] | [Quick bullet points] |

### What You Built
- [Deliverable 1]
- [Deliverable 2]

### Next Steps
Continue to [Next Chapter](./next-chapter.md) to learn about [next topic].

</div>


---

## 4. Sidebar Configuration Specification

### 4.1 Module-Level _category_.json

Each module folder MUST contain a _category_.json file:

json
{
  "label": "[Icon] [Module Name]",
  "position": [number],
  "link": {
    "type": "generated-index",
    "description": "[1-2 sentence description of module purpose and what students will learn]",
    "slug": "/[module-slug]"
  },
  "collapsed": false,
  "className": "category-module"
}


*Examples*:

*00-introduction/category.json*:
json
{
  "label": "📚 Introduction",
  "position": 0,
  "link": {
    "type": "generated-index",
    "description": "Welcome to Physical AI & Humanoid Robotics. Learn what this course covers, prerequisites, hardware requirements,
and how to navigate the textbook effectively.",
    "slug": "/introduction"
  },
  "collapsed": false
}


*01-ros2/category.json*:
json
{
  "label": "🧠 ROS 2: The Robotic Nervous System",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Master ROS 2, the middleware that powers modern robotics. Learn nodes, topics, services, actions, and how to
build Python packages for robot control.",
    "slug": "/ros2"
  },
  "collapsed": false
}


*02-simulation/category.json*:
json
{
  "label": "🎮 Simulation: The Digital Twin",
  "position": 2,
  "link": {
    "type": "generated-index",
    "description": "Build realistic robot simulations with Gazebo. Learn URDF/SDF formats, sensor integration, physics engines, and
world building for testing before deployment.",
    "slug": "/simulation"
  },
  "collapsed": false
}


*03-isaac/category.json*:
json
{
  "label": "🤖 NVIDIA Isaac: The AI Brain",
  "position": 3,
  "link": {
    "type": "generated-index",
    "description": "Harness NVIDIA Isaac Sim and Isaac ROS for AI-powered robotics. Master visual SLAM, perception pipelines, and
sim-to-real transfer techniques.",
    "slug": "/isaac"
  },
  "collapsed": false
}


*04-vla/category.json*:
json
{
  "label": "🗣 VLA: Vision-Language-Action",
  "position": 4,
  "link": {
    "type": "generated-index",
    "description": "Connect language models to robot actions. Build voice-controlled robots using OpenAI Whisper, LLM planning, and
multi-modal interaction systems.",
    "slug": "/vla"
  },
  "collapsed": false
}


*05-capstone/category.json*:
json
{
  "label": "🎯 Capstone: Autonomous Humanoid",
  "position": 5,
  "link": {
    "type": "generated-index",
    "description": "Integrate everything into a complete autonomous humanoid robot. Voice commands, navigation, object detection, and
 manipulation in a unified system.",
    "slug": "/capstone"
  },
  "collapsed": false
}


---

## 5. Content Standards Specification

### 5.1 Writing Style Guidelines

*Tone & Voice*:
- Professional yet conversational
- Active voice, second person ("you will learn")
- Mentor-like guidance without condescension
- Assume intelligence, explain complexity

*Sentence Structure*:
- Average 15-20 words per sentence
- Vary sentence length for readability
- Use short sentences for emphasis
- Break complex ideas into digestible chunks

*Paragraph Length*:
- 3-5 sentences per paragraph
- One main idea per paragraph
- Use transition sentences between paragraphs

*Technical Accuracy*:
- All code must be tested and working
- Use latest stable versions (ROS 2 Humble)
- Include version numbers for dependencies
- Cite official documentation for claims

---

### 5.2 Code Block Standards

*Python Code*:
python title="filename.py"
"""
Module docstring explaining purpose
"""

import ros2_package
from typing import List, Optional

class ExampleNode:
    """Class docstring"""

    def __init__(self, param: str) -> None:
        """
        Initialize node.

        Args:
            param: Description of parameter
        """
        self.param = param

    def method_name(self) -> None:
        """Method docstring with explanation"""
        # Inline comments for non-obvious logic
        pass


*Bash Commands*:
bash
# Install ROS 2 package
sudo apt update
sudo apt install ros-humble-package-name

# Build workspace
cd ~/ros2_ws
colcon build --packages-select my_package

# Source and run
source install/setup.bash
ros2 run my_package my_node


*XML/URDF*:
xml title="robot.urdf"
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Robot description with comments -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>


*Expected Output* (when showing command results):
bash
# Command
ros2 topic list

# Expected output:
/parameter_events
/rosout
/robot/cmd_vel
/robot/odom


---

### 5.3 Callout Box Usage

*Note* (General information):
markdown
:::note Definition
A **node** in ROS 2 is a process that performs computation. Each node should have a single, well-defined purpose.
:::


*Tip* (Best practices):
markdown
:::tip Best Practice
Always namespace your topics to avoid conflicts:
- Good: `/robot1/cmd_vel`
- Bad: `/cmd_vel`
:::


*Info* (Important context):
markdown
:::info ROS 2 vs ROS 1
ROS 2 uses DDS (Data Distribution Service) instead of a central master node, enabling better scalability and real-time performance.
:::


*Caution* (Common mistakes):
markdown
:::caution Common Mistake
Don't forget to source your workspace after building:
bash
source install/setup.bash

Missing this step causes "package not found" errors.
:::


*Danger* (Critical warnings):
markdown
:::danger Safety Warning
Never run untested motion commands on real hardware without:
1. Emergency stop button ready
2. Clear workspace
3. Proper safety barriers
:::


---

### 5.4 Diagram Specifications

*Mermaid Flow Diagrams*:
markdown
mermaid
graph LR
    A[User Input] --> B[ROS 2 Node]
    B --> C[Gazebo Simulation]
    C --> D[Sensor Data]
    D --> B
    B --> E[Robot Action]
```
```

*ASCII Diagrams* (for terminal concepts):

┌─────────────────────────────────────┐
│         ROS 2 Architecture          │
├─────────────────────────────────────┤
│  ┌───────┐      ┌───────┐          │
│  │ Node1 │─────▶│ Node2 │          │
│  └───────┘      └───────┘          │
│       │              │              │
│       └──────┬───────┘              │
│              ▼                      │
│         ┌─────────┐                │
│         │ Topic   │                │
│         └─────────┘                │
└─────────────────────────────────────┘


*Requirements*:
- All diagrams must have descriptive captions
- Use consistent color schemes
- Keep diagrams simple (max 7-10 elements)
- Provide alt text for accessibility

---

### 5.5 Table Standards

*Comparison Tables*:
markdown
| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | TCPROS | DDS |
| Real-time | Limited | Built-in |
| Python | 2.7/3.x | 3.6+ |
| Lifecycle | No | Yes |


*Parameter Tables*:
markdown
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_rate` | float | 10.0 | Sensor update frequency (Hz) |
| `frame_id` | string | "base_link" | TF frame name |
| `topic_name` | string | "/scan" | Published topic |


*Requirements*:
- Always include header row
- Align columns for readability
- Use code formatting for technical values
- Keep descriptions concise (1-2 sentences max)

---

## 6. Exercise Specification

### 6.1 Exercise Structure

Each chapter MUST include 2-4 exercises following this template:

markdown
:::tip Exercise [Module].[Chapter].[Number]: [Exercise Name]

**Objective**: [One sentence describing what to build]

**Difficulty**: ⭐[Easy/Medium/Hard]

**Time Estimate**: [X-Y minutes]

**Requirements**:
1. [Specific, testable requirement 1]
2. [Specific, testable requirement 2]
3. [Specific, testable requirement 3]

**Starter Code**:
python title="exercise_template.py"
# Starter template provided to student
# TODO: Complete the implementation


**Deliverable**: [Exactly what to submit or demonstrate]

**Success Criteria**:
- [ ] [Checkable criterion 1]
- [ ] [Checkable criterion 2]
- [ ] [Checkable criterion 3]

**Test Commands**:
bash
# Commands to verify solution works
ros2 run my_package test_node
ros2 topic echo /output_topic


**Expected Output**:

[Show what successful execution looks like]


**Challenge** (Optional): [Advanced extension for fast learners]

**Hints**:
<details>
<summary>Click for hint 1</summary>

[Progressive hint that doesn't give away answer]
</details>

<details>
<summary>Click for hint 2</summary>

[More detailed hint]
</details>
:::


### 6.2 Exercise Difficulty Levels

*⭐ Easy* (5-15 minutes):
- Modify existing code
- Single concept application
- Clear, step-by-step guidance

*⭐⭐ Medium* (15-30 minutes):
- Combine 2-3 concepts
- Some independent problem-solving
- Reference previous examples

*⭐⭐⭐ Hard* (30-60 minutes):
- Integrate multiple modules
- Requires architectural thinking
- Minimal scaffolding provided

---

## 7. Troubleshooting Section Specification

### 7.1 Standard Format

Each chapter MUST include troubleshooting section:

markdown
## Common Issues and Debugging

:::caution Common Problems

**Problem 1: [Clear, specific issue description]**

**Symptoms**:
- [What the user sees]
- [Error messages with exact text]
- [Unexpected behavior]

**Cause**: [Why this happens]

**Solution**:
bash
# Step-by-step fix commands


**Verification**:
bash
# Commands to confirm fix worked


---

**Problem 2: [Next issue]**

[Same structure]
:::


### 7.2 Required Problems Per Chapter

Minimum problems to address:
- Installation/setup issues: 2-3
- Runtime errors: 2-3
- Configuration problems: 1-2
- Performance issues: 1 (if applicable)

---

## 8. Module Index Page Specification

### 8.1 Module Index Template

Each module folder MUST have an index.md:

markdown
---
sidebar_position: 0
title: "[Module Name] Overview"
---

# [Module Icon] [Module Name]

## Module Overview

[2-3 paragraphs explaining module purpose, importance, and what students will achieve]

---

## Learning Path

mermaid
graph LR
    A[Chapter 1] --> B[Chapter 2]
    B --> C[Chapter 3]
    C --> D[Chapter 4]
    D --> E[Chapter 5]


---

## What You'll Learn

By completing this module, you will:

- ✅ [Specific skill 1]
- ✅ [Specific skill 2]
- ✅ [Specific skill 3]
- ✅ [Specific skill 4]

---

## Module Chapters

### [Chapter 1 Name](./01-chapter.md)
[2-3 sentence description]
- **Time**: [X hours]
- **Difficulty**: ⭐⭐

### [Chapter 2 Name](./02-chapter.md)
[2-3 sentence description]
- **Time**: [X hours]
- **Difficulty**: ⭐⭐

[Continue for all chapters...]

---

## Prerequisites

**Required Knowledge**:
- [Prerequisite 1]
- [Prerequisite 2]

**Required Software**:
- [Software 1] (version X.Y.Z)
- [Software 2] (version X.Y.Z)

**Hardware Requirements**:
- [Hardware requirement if applicable]

---

## Module Project

At the end of this module, you'll build: **[Project Name]**

[2-3 sentences describing the capstone project for this module]

**Features**:
- [Feature 1]
- [Feature 2]
- [Feature 3]

---

## Time Commitment

| Chapter | Topic | Estimated Time |
|---------|-------|----------------|
| 1 | [Topic] | [X hours] |
| 2 | [Topic] | [X hours] |
| ... | ... | ... |
| **Total** | | **[X-Y hours]** |

---

**Ready to Start?** → [Begin Chapter 1](./01-chapter.md)


---

## 9. Custom CSS Specifications

### 9.1 Required Styles (in static/css/custom.css)

css
/* ============================================
   READING TIME COMPONENT
   ============================================ */
.reading-time-badge {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border-radius: 20px;
  font-size: 14px;
  font-weight: 600;
  margin-bottom: 20px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  animation: fadeInDown 0.5s ease-in-out;
}

.reading-time-icon {
  font-size: 18px;
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0%, 100% { transform: scale(1); }
  50% { transform: scale(1.1); }
}

/* ============================================
   VIEW TOGGLE COMPONENT
   ============================================ */
.page-header {
  margin-bottom: 30px;
  padding-bottom: 15px;
  border-bottom: 2px solid #e0e0e0;
  animation: fadeIn 0.5s ease-in-out;
}

.header-controls {
  display: flex;
  gap: 12px;
  flex-wrap: wrap;
}

.view-toggle {
  padding: 10px 20px;
  border: 2px solid #667eea;
  background: white;
  color: #667eea;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.view-toggle:hover {
  background: #f0f4ff;
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.15);
}

.view-toggle.active {
  background: #667eea;
  color: white;
  box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
}

/* ============================================
   VIEW SWITCHING LOGIC
   ============================================ */
/* Default: Show full content, hide summary */
.full-content {
  display: block;
}

.summary-content {
  display: none;
  background: #f8f9fa;
  border-left: 4px solid #667eea;
  padding: 20px;
  border-radius: 8px;
  margin-top: 20px;
}

/* When summary view is active */
body[data-view="summary"] .full-content {
  display: none;
}

body[data-view="summary"] .summary-content {
  display: block;
}

/* ============================================
   CHAPTER HEADER STYLING
   ============================================ */
.chapter-meta {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 15px;
  padding: 20px;
  background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
  border-radius: 10px;
  margin: 20px 0;
}

.chapter-meta strong {
  color: #667eea;
}

/* ============================================
   EXERCISE STYLING
   ============================================ */
.theme-admonition-tip {
  border-left-color: #00b894;
}

.theme-admonition-tip .admonition-heading h5 {
  color: #00b894;
}

/* ============================================
   SIDEBAR ENHANCEMENTS
   ============================================ */
.theme-doc-sidebar-item-category {
  margin-bottom: 10px;
}

.theme-doc-sidebar-item-category-level-1 > .menu__list-item-collapsible {
  font-weight: 700;
  font-size: 1.1em;
  border-bottom: 2px solid #e0e0e0;
  padding-bottom: 8px;
  margin-bottom: 8px;
}

/* ============================================
   CODE BLOCK ENHANCEMENTS
   ============================================ */
pre[class*="language-"] {
  border-radius: 8px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

code[class*="language-"] {
  font-family: 'Fira Code', 'Monaco', monospace;
}

/* ============================================
   TABLE STYLING
   ============================================ */
table {
  width: 100%;
  border-collapse: collapse;
  margin: 20px 0;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  border-radius: 8px;
  overflow: hidden;
}

thead {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

thead th {
  padding: 12px;
  text-align: left;
  font-weight: 600;
}

tbody tr {
  border-bottom: 1px solid #e0e0e0;
  transition: background 0.2s ease;
}

tbody tr:hover {
  background: #f8f9fa;
}

tbody td {
  padding: 12px;
}

/* ============================================
   ANIMATIONS
   ============================================ */
@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}

@keyframes fadeInDown {
  from {
    opacity: 0;
    transform: translateY(-10px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* ============================================
   RESPONSIVE DESIGN
   ============================================ */
@media (max-width: 768px) {
  .reading-time-badge {
    font-size: 12px;
    padding: 6px 12px;
  }

  .view-toggle {
    font-size: 12px;
    padding: 8px 16px;
  }

  .chapter-meta {
    grid-template-columns: 1fr;
  }
}


---
