# Chapter Personalization Feature

The Chapter Personalization feature allows logged-in users to dynamically adapt textbook content based on their background, experience level, and hardware capabilities. This AI-powered system personalizes content in real-time while preserving the original learning objectives.

## How It Works

1. **User Profile Integration**: The system uses your profile information including:
   - Software background level (beginner, intermediate, advanced)
   - Hardware background and experience
   - Programming languages familiarity
   - Robotics and AI/ML experience levels
   - ROS experience (true/false)
   - GPU access availability (true/false)
   - Learning goals

2. **AI-Powered Personalization**: The content is processed through our RAG (Retrieval-Augmented Generation) system which adapts the content based on your profile.

3. **Real-time Processing**: Content is personalized on-demand without modifying source files.

## Using the Feature

### Personalizing Content
1. Ensure you are logged in to your account
2. Navigate to any chapter you want to personalize
3. Click the **"🎯 Personalize this Chapter"** button at the top of the content
4. Wait for the personalization to complete (typically under 3 seconds)
5. The content will be updated to match your background and experience level

### Resetting to Original
- Click the **"✓ Personalized"** button to revert to the original content

## Personalization Rules

The AI applies the following personalization rules based on your profile:

### Experience Level Adaptation
- **Beginner**: More detailed explanations, step-by-step guidance, concrete examples, and prerequisite explanations
- **Intermediate**: Balanced approach with moderate complexity and some examples
- **Advanced**: Concise explanations with advanced concepts, optimizations, and cutting-edge techniques

### Hardware Constraint Handling
- **No GPU Access**: Provides CPU-friendly alternatives, mentions computational complexity, suggests smaller models or datasets
- **GPU Access**: Includes GPU optimization tips, mentions parallel processing opportunities, suggests computationally intensive approaches

### ROS Experience Adaptation
- **No ROS Experience**: Includes basic ROS concepts, explains ROS terminology, provides gentle introduction to ROS architecture
- **ROS Experience**: Skips basic ROS concepts, focuses on advanced ROS features, advanced ROS 2 concepts, and performance optimizations

### Programming Language Adaptation
- Adapts code examples based on your familiar programming languages (Python, C++, etc.)

### Robotics and AI/ML Experience
- Modifies mathematical complexity based on your AI/ML experience level
- Adjusts robotics concept complexity based on your robotics experience

## Technical Architecture

### Frontend (Docusaurus)
- **PersonalizeButton.tsx**: React component with loading states, animations, and error handling
- **personalization.ts**: API client service with fallback mechanisms
- Orange & white professional styling as specified

### Backend (FastAPI)
- **Personalization Service**: Reuses existing RAG infrastructure
- **API Endpoint**: `/api/personalize/chapter` with authentication validation
- **Template System**: Structured AI prompts based on user profile
- **Chunking Logic**: Processes large content in sections for better performance

### Security
- Validates user sessions through the auth-backend
- No authentication logic duplication (uses existing Better Auth system)
- Protected by the same security measures as the main application

## Performance
- Target response time: Under 3 seconds
- Content larger than 2KB is processed in chunks for better performance
- Includes timeout handling and retry logic
- Comprehensive error logging and fallback mechanisms

## Error Handling
- Graceful fallback to original content if personalization fails
- Detailed error logging for debugging
- User-friendly error states in the UI

## Accessibility
- Keyboard navigable controls
- Sufficient color contrast for the orange & white theme
- Clear labeling and instructions
- Loading states for users with disabilities