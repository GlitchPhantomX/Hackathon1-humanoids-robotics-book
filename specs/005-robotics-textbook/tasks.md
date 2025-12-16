# Implementation Tasks: Physical AI & Humanoid Robotics Textbook

**Feature**: Physical AI & Humanoid Robotics Textbook
**Branch**: `005-robotics-textbook`
**Date**: 2025-12-10
**Status**: Phase 1 Complete (Setup already done) - Starting from Phase 2

## Summary

Implementation tasks for creating a comprehensive, professional textbook using Docusaurus that teaches Physical AI & Humanoid Robotics. The book follows a 6-module structure with interactive features including reading time indicators and dual-view (Full Lesson/Summary) functionality.

**Note**: Phase 1 (Setup) has been completed. This document focuses on Phase 2 onwards.

---

## Phase 2: Foundational Tasks

### Story Goal
Implement the core components and styling that will be used throughout the textbook.

### Independent Test Criteria
- ReadingTime component renders with correct styling
- ViewToggle component toggles between views correctly
- Custom CSS styles are applied consistently

### Tasks

- [X] T009 Verify/Create src/components/ directory in physical-ai-robotics-textbook/docusaurus/src/components/
- [X] T010 [P] Implement ReadingTime component with requirements from spec in physical-ai-robotics-textbook/docusaurus/src/components/ReadingTime.jsx
- [X] T011 [P] Implement ViewToggle component with requirements from spec in physical-ai-robotics-textbook/docusaurus/src/components/ViewToggle.jsx
- [X] T012 Verify/Create static/css/ directory in physical-ai-robotics-textbook/docusaurus/static/css/
- [X] T013 [P] Implement custom CSS with requirements from spec in physical-ai-robotics-textbook/docusaurus/static/css/custom.css
- [X] T014 [P] Create base index.md file in physical-ai-robotics-textbook/docusaurus/docs/index.md
- [X] T015 [P] Create base _category_.json file in physical-ai-robotics-textbook/docusaurus/docs/_category_.json
- [X] T016 Test components render correctly in development environment

---

## Phase 3: [US1] Access Comprehensive Robotics Textbook Content

### Story Goal
Create the foundational content structure with 6 modules and implement the core textbook functionality to provide comprehensive learning content covering all specified topics (ROS 2, simulation, NVIDIA Isaac, VLA, and capstone integration) with interactive features like reading time indicators and dual-view functionality.

### Independent Test Criteria
- All 6 modules are accessible with proper navigation
- ReadingTime component displays at the top of each chapter
- ViewToggle component allows switching between Full Lesson and Summary views
- Content follows the exact template structure from the specification

### Module Directory Creation (Parallel)

- [X] T017 [P] [US1] Create 00-introduction directory structure in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/
- [X] T018 [P] [US1] Create 01-ros2 directory structure in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/
- [X] T019 [P] [US1] Create 02-simulation directory structure in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/
- [X] T020 [P] [US1] Create 03-isaac directory structure in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/
- [X] T021 [P] [US1] Create 04-vla directory structure in physical-ai-robotics-textbook/docusaurus/docs/04-vla/
- [X] T022 [P] [US1] Create 05-capstone directory structure in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/

### Module Category Files (Parallel)

- [X] T023 [P] [US1] Create introduction module _category_.json in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/_category_.json
- [X] T024 [P] [US1] Create ros2 module _category_.json in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/_category_.json
- [X] T025 [P] [US1] Create simulation module _category_.json in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/_category_.json
- [X] T026 [P] [US1] Create isaac module _category_.json in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/_category_.json
- [X] T027 [P] [US1] Create vla module _category_.json in physical-ai-robotics-textbook/docusaurus/docs/04-vla/_category_.json
- [X] T028 [P] [US1] Create capstone module _category_.json in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/_category_.json

### Module Index Files (Parallel)

- [X] T029 [P] [US1] Create introduction module index.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/index.md
- [X] T030 [P] [US1] Create ros2 module index.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/index.md
- [X] T031 [P] [US1] Create simulation module index.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/index.md
- [X] T032 [P] [US1] Create isaac module index.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/index.md
- [X] T033 [P] [US1] Create vla module index.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/index.md
- [X] T034 [P] [US1] Create capstone module index.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/index.md

### Introduction Module Chapters (Parallel)

- [X] T035 [P] [US1] Create 01-welcome.md chapter in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/01-welcome.md
- [X] T036 [P] [US1] Create 02-prerequisites.md chapter in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/02-prerequisites.md
- [X] T037 [P] [US1] Create 03-hardware-requirements.md chapter in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/03-hardware-requirements.md
- [X] T038 [P] [US1] Create 04-how-to-use.md chapter in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/04-how-to-use.md
- [X] T039 [P] [US1] Create 05-syllabus.md chapter in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/05-syllabus.md

### ROS 2 Module Chapters (Parallel)

- [ ] T040 [P] [US1] Create 01-architecture.md chapter in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/01-architecture.md
- [ ] T041 [P] [US1] Create 02-nodes-topics.md chapter in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/02-nodes-topics.md
- [X] T042 [P] [US1] Create 03-services-actions.md chapter in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/03-services-actions.md
- [X] T043 [P] [US1] Create 04-python-packages.md chapter in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/04-python-packages.md
- [X] T044 [P] [US1] Create 05-urdf-humanoids.md chapter in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/05-urdf-humanoids.md
- [X] T045 [P] [US1] Create 06-launch-files.md chapter in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/06-launch-files.md

### Simulation Module Chapters (Parallel)

- [ ] T046 [P] [US1] Create 01-gazebo-intro.md chapter in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/01-gazebo-intro.md
- [ ] T047 [P] [US1] Create 02-urdf-sdf.md chapter in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/02-urdf-sdf.md
- [ ] T048 [P] [US1] Create 03-sensors-plugins.md chapter in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/03-sensors-plugins.md
- [ ] T049 [P] [US1] Create 04-world-building.md chapter in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/04-world-building.md
- [ ] T050 [P] [US1] Create 05-ros2-integration.md chapter in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/05-ros2-integration.md
- [ ] T051 [P] [US1] Create 06-advanced-simulation.md chapter in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/06-advanced-simulation.md

### Isaac Module Chapters (Parallel)

- [ ] T052 [P] [US1] Create 01-isaac-sim.md chapter in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/01-isaac-sim.md
- [ ] T053 [P] [US1] Create 02-isaac-ros.md chapter in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/02-isaac-ros.md
- [ ] T054 [P] [US1] Create 03-vslam-navigation.md chapter in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/03-vslam-navigation.md
- [ ] T055 [P] [US1] Create 04-perception.md chapter in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/04-perception.md
- [ ] T056 [P] [US1] Create 05-sim-to-real.md chapter in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/05-sim-to-real.md

### VLA Module Chapters (Parallel)

- [ ] T057 [P] [US1] Create 01-voice-to-action.md chapter in physical-ai-robotics-textbook/docusaurus/docs/04-vla/01-voice-to-action.md
- [ ] T058 [P] [US1] Create 02-llm-planning.md chapter in physical-ai-robotics-textbook/docusaurus/docs/04-vla/02-llm-planning.md
- [ ] T059 [P] [US1] Create 03-natural-language.md chapter in physical-ai-robotics-textbook/docusaurus/docs/04-vla/03-natural-language.md
- [ ] T060 [P] [US1] Create 04-multimodal.md chapter in physical-ai-robotics-textbook/docusaurus/docs/04-vla/04-multimodal.md

### Capstone Module Chapters (Parallel)

- [ ] T061 [P] [US1] Create 01-project-overview.md chapter in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/01-project-overview.md
- [ ] T062 [P] [US1] Create 02-architecture.md chapter in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/02-architecture.md
- [ ] T063 [P] [US1] Create 03-voice-system.md chapter in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/03-voice-system.md
- [ ] T064 [P] [US1] Create 04-navigation.md chapter in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/04-navigation.md
- [ ] T065 [P] [US1] Create 05-manipulation.md chapter in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/05-manipulation.md
- [ ] T066 [P] [US1] Create 06-integration.md chapter in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/06-integration.md

### Content Verification

- [X] T067 [P] [US1] Verify all chapters include ReadingTime component with calculated minutes
- [X] T068 [P] [US1] Verify all chapters include ViewToggle component
- [X] T069 [P] [US1] Verify all chapters follow the exact template structure from spec
- [ ] T070 [US1] Test navigation between all modules and chapters

---

## Phase 4: [US2] Interactive Learning Experience with Reading Time Indicators

### Story Goal
Implement the reading time indicator functionality that allows learners to plan their study sessions effectively and track their progress through the material by providing accurate time estimates at the top of each chapter.

### Independent Test Criteria
- Reading time indicators are displayed prominently at the top of each chapter
- Time estimates are accurate based on content complexity
- Component styling matches the design specifications

### Reading Time Calculations (Parallel per Module)

- [ ] T071 [P] [US2] Calculate reading times for all introduction module chapters
- [X] T072 [P] [US2] Calculate reading times for all ros2 module chapters
- [ ] T073 [P] [US2] Calculate reading times for all simulation module chapters
- [ ] T074 [P] [US2] Calculate reading times for all isaac module chapters
- [ ] T075 [P] [US2] Calculate reading times for all vla module chapters
- [ ] T076 [P] [US2] Calculate reading times for all capstone module chapters

### Reading Time Integration (Parallel)

- [X] T077 [P] [US2] Update all chapter files with calculated reading times in ReadingTime component
- [X] T078 [P] [US2] Verify ReadingTime component displays correctly on all chapters
- [X] T079 [P] [US2] Test ReadingTime component styling matches spec requirements
- [X] T080 [US2] Validate reading time accuracy against actual content length

---

## Phase 5: [US3] Hands-on Exercise and Troubleshooting Support

### Story Goal
Implement hands-on exercises with clear success criteria and troubleshooting guidance so that users can apply theoretical concepts and resolve common issues during implementation.

### Independent Test Criteria
- All chapters include 2-4 exercises with clear objectives and requirements
- Exercises have success criteria and test commands as specified
- Troubleshooting sections address common problems with solutions
- Content follows the exact exercise and troubleshooting templates from the specification

### Add Exercises (Parallel per Module)

- [ ] T081 [P] [US3] Add exercises to 01-welcome.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/01-welcome.md
- [ ] T082 [P] [US3] Add exercises to 02-prerequisites.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/02-prerequisites.md
- [ ] T083 [P] [US3] Add exercises to 03-hardware-requirements.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/03-hardware-requirements.md
- [ ] T084 [P] [US3] Add exercises to 04-how-to-use.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/04-how-to-use.md
- [ ] T085 [P] [US3] Add exercises to 05-syllabus.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/05-syllabus.md
- [ ] T086 [P] [US3] Add exercises to 01-architecture.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/01-architecture.md
- [ ] T087 [P] [US3] Add exercises to 02-nodes-topics.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/02-nodes-topics.md
- [X] T088 [P] [US3] Add exercises to 03-services-actions.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/03-services-actions.md
- [X] T089 [P] [US3] Add exercises to 04-python-packages.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/04-python-packages.md
- [X] T090 [P] [US3] Add exercises to 05-urdf-humanoids.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/05-urdf-humanoids.md
- [X] T091 [P] [US3] Add exercises to 06-launch-files.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/06-launch-files.md
- [ ] T092 [P] [US3] Add exercises to 01-gazebo-intro.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/01-gazebo-intro.md
- [ ] T093 [P] [US3] Add exercises to 02-urdf-sdf.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/02-urdf-sdf.md
- [ ] T094 [P] [US3] Add exercises to 03-sensors-plugins.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/03-sensors-plugins.md
- [ ] T095 [P] [US3] Add exercises to 04-world-building.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/04-world-building.md
- [ ] T096 [P] [US3] Add exercises to 05-ros2-integration.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/05-ros2-integration.md
- [ ] T097 [P] [US3] Add exercises to 06-advanced-simulation.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/06-advanced-simulation.md
- [ ] T098 [P] [US3] Add exercises to 01-isaac-sim.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/01-isaac-sim.md
- [ ] T099 [P] [US3] Add exercises to 02-isaac-ros.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/02-isaac-ros.md
- [ ] T100 [P] [US3] Add exercises to 03-vslam-navigation.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/03-vslam-navigation.md
- [ ] T101 [P] [US3] Add exercises to 04-perception.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/04-perception.md
- [ ] T102 [P] [US3] Add exercises to 05-sim-to-real.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/05-sim-to-real.md
- [ ] T103 [P] [US3] Add exercises to 01-voice-to-action.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/01-voice-to-action.md
- [ ] T104 [P] [US3] Add exercises to 02-llm-planning.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/02-llm-planning.md
- [ ] T105 [P] [US3] Add exercises to 03-natural-language.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/03-natural-language.md
- [ ] T106 [P] [US3] Add exercises to 04-multimodal.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/04-multimodal.md
- [X] T107 [P] [US3] Add exercises to 01-project-overview.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/01-project-overview.md
- [X] T108 [P] [US3] Add exercises to 02-architecture.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/02-architecture.md
- [X] T109 [P] [US3] Add exercises to 03-voice-system.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/03-voice-system.md
- [X] T110 [P] [US3] Add exercises to 04-navigation.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/04-navigation.md
- [X] T111 [P] [US3] Add exercises to 05-manipulation.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/05-manipulation.md
- [X] T112 [P] [US3] Add exercises to 06-integration.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/06-integration.md

### Add Troubleshooting Sections (Parallel per Module)

- [X] T113 [P] [US3] Add troubleshooting sections to 01-welcome.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/01-welcome.md
- [X] T114 [P] [US3] Add troubleshooting sections to 02-prerequisites.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/02-prerequisites.md
- [X] T115 [P] [US3] Add troubleshooting sections to 03-hardware-requirements.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/03-hardware-requirements.md
- [X] T116 [P] [US3] Add troubleshooting sections to 04-how-to-use.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/04-how-to-use.md
- [X] T117 [P] [US3] Add troubleshooting sections to 05-syllabus.md in physical-ai-robotics-textbook/docusaurus/docs/00-introduction/05-syllabus.md
- [X] T118 [P] [US3] Add troubleshooting sections to 01-architecture.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/01-architecture.md
- [X] T119 [P] [US3] Add troubleshooting sections to 02-nodes-topics.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/02-nodes-topics.md
- [X] T120 [P] [US3] Add troubleshooting sections to 03-services-actions.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/03-services-actions.md
- [X] T121 [P] [US3] Add troubleshooting sections to 04-python-packages.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/04-python-packages.md
- [X] T122 [P] [US3] Add troubleshooting sections to 05-urdf-humanoids.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/05-urdf-humanoids.md
- [X] T123 [P] [US3] Add troubleshooting sections to 06-launch-files.md in physical-ai-robotics-textbook/docusaurus/docs/01-ros2/06-launch-files.md
- [X] T124 [P] [US3] Add troubleshooting sections to 01-gazebo-intro.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/01-gazebo-intro.md
- [X] T125 [P] [US3] Add troubleshooting sections to 02-urdf-sdf.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/02-urdf-sdf.md
- [X] T126 [P] [US3] Add troubleshooting sections to 03-sensors-plugins.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/03-sensors-plugins.md
- [X] T127 [P] [US3] Add troubleshooting sections to 04-world-building.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/04-world-building.md
- [X] T128 [P] [US3] Add troubleshooting sections to 05-ros2-integration.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/05-ros2-integration.md
- [X] T129 [P] [US3] Add troubleshooting sections to 06-advanced-simulation.md in physical-ai-robotics-textbook/docusaurus/docs/02-simulation/06-advanced-simulation.md
- [X] T130 [P] [US3] Add troubleshooting sections to 01-isaac-sim.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/01-isaac-sim.md
- [X] T131 [P] [US3] Add troubleshooting sections to 02-isaac-ros.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/02-isaac-ros.md
- [X] T132 [P] [US3] Add troubleshooting sections to 03-vslam-navigation.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/03-vslam-navigation.md
- [X] T133 [P] [US3] Add troubleshooting sections to 04-perception.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/04-perception.md
- [X] T134 [P] [US3] Add troubleshooting sections to 05-sim-to-real.md in physical-ai-robotics-textbook/docusaurus/docs/03-isaac/05-sim-to-real.md
- [X] T135 [P] [US3] Add troubleshooting sections to 01-voice-to-action.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/01-voice-to-action.md
- [X] T136 [P] [US3] Add troubleshooting sections to 02-llm-planning.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/02-llm-planning.md
- [X] T137 [P] [US3] Add troubleshooting sections to 03-natural-language.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/03-natural-language.md
- [X] T138 [P] [US3] Add troubleshooting sections to 04-multimodal.md in physical-ai-robotics-textbook/docusaurus/docs/04-vla/04-multimodal.md
- [X] T139 [P] [US3] Add troubleshooting sections to 01-project-overview.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/01-project-overview.md
- [X] T140 [P] [US3] Add troubleshooting sections to 02-architecture.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/02-architecture.md
- [X] T141 [P] [US3] Add troubleshooting sections to 03-voice-system.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/03-voice-system.md
- [X] T142 [P] [US3] Add troubleshooting sections to 04-navigation.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/04-navigation.md
- [X] T143 [P] [US3] Add troubleshooting sections to 05-manipulation.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/05-manipulation.md
- [X] T144 [P] [US3] Add troubleshooting sections to 06-integration.md in physical-ai-robotics-textbook/docusaurus/docs/05-capstone/06-integration.md

### Verification

- [X] T145 [US3] Verify all exercises follow the spec template with objectives, requirements, deliverables, and test commands
- [X] T146 [US3] Verify all troubleshooting sections follow the spec template with problems, symptoms, causes, and solutions
- [X] T147 [US3] Test exercise and troubleshooting functionality in development environment

---

## Phase 6: Polish & Cross-Cutting Concerns

### Story Goal
Finalize the textbook with additional resources, proper navigation, and ensure all content meets the quality standards specified in the content standards section.

### Independent Test Criteria
- All chapters include proper navigation links
- Additional resources sections are properly formatted
- Content follows writing style guidelines
- Code examples follow standards from the specification
- Site is properly configured for deployment

### Navigation & Resources (Parallel per Module)

- [X] T148 Add proper navigation links to all chapters (Next/Previous) in all markdown files
- [X] T149 [P] Add additional resources sections to all chapters in all markdown files
- [X] T150 [P] Verify all code examples follow the Python, Bash, XML/URDF standards from spec in all markdown files
- [X] T151 [P] Verify all callout boxes (Note, Tip, Info, Caution, Danger) are properly formatted in all markdown files
- [X] T152 [P] Verify all diagrams follow the specification requirements in all markdown files
- [X] T153 [P] Verify all tables follow the comparison and parameter table standards in all markdown files

### Configuration & Build (Sequential)

- [X] T154 [P] Add proper metadata and frontmatter to all chapters following spec requirements
- [X] T155 [P] Update docusaurus.config.mjs with complete configuration for production in physical-ai-robotics-textbook/docusaurus/docusaurus.config.mjs
- [X] T156 [P] Update sidebars.js to include all chapters in proper order in physical-ai-robotics-textbook/docusaurus/sidebars.js
- [X] T157 Test full navigation flow through all modules and chapters
- [X] T158 Verify responsive design works on different screen sizes
- [X] T159 Run build process to ensure site compiles without errors
- [X] T160 Conduct final review of all content for consistency and quality
- [X] T161 Update README with deployment instructions

---

## Dependencies

### User Story Completion Order
1. **US1** (Access Comprehensive Content) - Foundation for all other stories
2. **US2** (Reading Time Indicators) - Depends on US1 content structure
3. **US3** (Exercises & Troubleshooting) - Depends on US1 content structure

### Parallel Execution Examples per Phase

#### Phase 2 - Sequential (Foundational):
- Components must be created before content uses them
- CSS must be prepared before testing

#### Phase 3 (US1) - Heavy Parallel Execution:
- Tasks T017-T022: Create all module directories in parallel
- Tasks T023-T028: Create all _category_.json files in parallel
- Tasks T029-T034: Create all module index files in parallel
- Chapter creation tasks within each module can run in parallel

#### Phase 4 (US2) - Parallel Execution:
- Tasks T071-T076: Calculate reading times per module in parallel
- Task T077: Update all chapters in parallel across modules

#### Phase 5 (US3) - Parallel Execution:
- Tasks T081-T112: Add exercises to all chapters in parallel
- Tasks T113-T144: Add troubleshooting to all chapters in parallel

#### Phase 6 - Mostly Parallel, Final Tasks Sequential:
- T148-T153: Can run in parallel across chapters
- T154-T161: Must run sequentially for proper configuration

---

## Implementation Strategy

### Quick Recovery Plan
Since Phase 1 was already complete but reset, follow this order:

**Step 1**: Restore Phase 2 components
- Create ReadingTime.jsx component
- Create ViewToggle.jsx component
- Update custom.css

**Step 2**: Rebuild Phase 3 structure (Highest Priority - MVP)
- Create all 6 module directories (T017-T022)
- Create all _category_.json files (T023-T028)
- Create all module index.md files (T029-T034)
- Create all chapter markdown files with basic template (T035-T066)

**Step 3**: Complete Phase 4 (Reading Times)
- Calculate and add reading times to all chapters

**Step 4**: Complete Phase 5 (Exercises)
- Add exercises and troubleshooting sections

**Step 5**: Polish Phase 6
- Update configuration, navigation, and resources

### MVP Scope (User Story 1)
The minimum viable product includes all of Phase 2 and Phase 3:
- ReadingTime and ViewToggle components working
- All 6 modules with proper navigation
- All chapters with basic template structure
- ReadingTime and ViewToggle components in all chapters
- Proper sidebar.js configuration

### Risk Mitigation
- Backup your current src/, docs/, and static/ directories
- Use git to track changes and enable rollback if needed
- Test components locally before deploying all chapters