# Data Model: Physical AI & Humanoid Robotics Textbook

## Content Entities

### Module
- **id**: string (e.g., "introduction", "ros2", "simulation", "isaac", "vla", "capstone")
- **title**: string (e.g., "Introduction", "ROS 2: The Robotic Nervous System")
- **position**: number (0-5 for ordering)
- **description**: string (brief overview of module content and learning objectives)
- **chapters**: Chapter[] (ordered list of chapters in the module)
- **prerequisites**: string[] (knowledge/skills required before starting)
- **estimatedTime**: string (e.g., "8-10 hours")
- **learningObjectives**: string[] (specific skills students will acquire)

### Chapter
- **id**: string (e.g., "introduction-welcome", "ros2-architecture")
- **title**: string (descriptive title of the chapter)
- **position**: number (within the module, 0-n)
- **moduleId**: string (reference to parent module)
- **learningObjectives**: string[] (specific objectives for this chapter)
- **prerequisites**: string[] (specific knowledge needed for this chapter)
- **estimatedTime**: string (reading time estimate)
- **content**: string (Markdown content of the chapter)
- **exercises**: Exercise[] (hands-on activities for the chapter)
- **troubleshooting**: TroubleshootingSection (common issues and solutions)

### Exercise
- **id**: string (e.g., "ros2-ex1", "simulation-ex2")
- **chapterId**: string (reference to parent chapter)
- **title**: string (descriptive name of the exercise)
- **objective**: string (what the student will build/achieve)
- **difficulty**: "easy" | "medium" | "hard" (⭐ rating system)
- **timeEstimate**: string (e.g., "15-20 minutes")
- **requirements**: string[] (specific, testable requirements)
- **starterCode**: string (optional template code to start with)
- **deliverable**: string (what the student submits/demonstrates)
- **testCommands**: string[] (commands to verify completion)
- **expectedOutput**: string (what successful execution looks like)
- **hints**: Hint[] (progressive guidance without giving away answer)

### Hint
- **id**: string (e.g., "hint-ros2-ex1-1")
- **exerciseId**: string (reference to parent exercise)
- **content**: string (progressive hint that doesn't give away the answer)
- **level**: number (order of revealing hints, 1-n)

### TroubleshootingSection
- **id**: string (e.g., "troubleshooting-ros2-chapter1")
- **chapterId**: string (reference to parent chapter)
- **problems**: TroubleshootingProblem[] (list of common issues)

### TroubleshootingProblem
- **id**: string (e.g., "problem-ros2-installation")
- **title**: string (clear, specific issue description)
- **symptoms**: string[] (what the user sees, including exact error messages)
- **cause**: string (why this happens)
- **solution**: string (step-by-step fix commands)
- **verification**: string[] (commands to confirm fix worked)

### Component
- **name**: string (e.g., "ReadingTime", "ViewToggle")
- **props**: Prop[] (required and optional properties)
- **description**: string (purpose and functionality)
- **implementation**: string (React component code)

### Prop
- **name**: string (property name)
- **type**: string (data type, e.g., "string", "number", "boolean")
- **required**: boolean (whether this prop is required)
- **description**: string (what this prop controls)

## Content Relationships

### Module contains Chapters
- One-to-many relationship: Each module contains multiple chapters
- Order maintained by position field
- Navigation hierarchy follows this relationship

### Chapter contains Exercises
- One-to-many relationship: Each chapter contains 2-4 exercises
- Exercises are specific to their chapter context
- Difficulty progression within chapter

### Chapter contains Troubleshooting
- One-to-one relationship: Each chapter has one troubleshooting section
- Section contains multiple problems specific to chapter content

### Exercise contains Hints
- One-to-many relationship: Each exercise may have multiple hints
- Hints are revealed progressively to support learning without giving away answers

## Content Validation Rules

### Module Validation
- Must have 1-6 chapters (minimum 1, maximum 6 as per spec)
- Title must be 1-100 characters
- Description must be 1-500 characters
- Position must be unique within the textbook (0-5)

### Chapter Validation
- Must have sidebar_position specified in frontmatter
- Title must be 1-200 characters
- Learning objectives must have 1-5 items
- Estimated time must follow "X-Y hours" format
- Must follow the exact template structure from spec

### Exercise Validation
- Must have 1-4 requirements
- Difficulty must be "easy", "medium", or "hard"
- Time estimate must follow "X-Y minutes" format
- Must include test commands for verification

### Troubleshooting Validation
- Must address 2-3 installation/setup issues per chapter
- Must address 2-3 runtime errors per chapter
- Must address 1-2 configuration problems per chapter
- Must address 1 performance issue if applicable

## State Transitions

### Content Creation Workflow
1. **Draft**: Initial content creation, internal review
2. **Review**: Peer review, technical accuracy verification
3. **Approved**: Content approved for publication
4. **Published**: Live on textbook website
5. **Archived**: Content deprecated (rare for textbook content)

### Exercise Status
1. **Design**: Exercise concept and requirements defined
2. **Implementation**: Exercise content and test commands created
3. **Validation**: Exercise tested and verified working
4. **Integrated**: Exercise added to chapter