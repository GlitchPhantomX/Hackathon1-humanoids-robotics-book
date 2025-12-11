# Data Model: Authentication System Frontend Redesign

## Overview
This document defines the data models and structures used in the authentication system frontend redesign for the Docusaurus-based robotics textbook website.

## User Profile Entity

### User Profile
- **id**: string (unique identifier from backend)
- **name**: string (user's display name)
- **email**: string (user's email address, validated format)
- **softwareBackground**: string (enum: 'beginner', 'intermediate', 'advanced', 'expert')
- **hardwareBackground**: string (enum: 'none', 'basic', 'intermediate', 'advanced')
- **programmingLanguages**: string[] (array of language strings: 'Python', 'JavaScript', 'C++', 'Java', 'ROS', etc.)
- **roboticsExperience**: string (enum: 'none', 'hobbyist', 'academic', 'professional')
- **aiMlExperience**: string (enum: 'none', 'basic', 'intermediate', 'advanced')
- **hasRosExperience**: boolean (whether user has ROS experience)
- **hasGpuAccess**: boolean (whether user has GPU access)
- **learningGoals**: string (textarea content describing user's learning goals)

### Validation Rules
- **id**: Required, unique, provided by backend
- **name**: Required, 1-100 characters
- **email**: Required, valid email format, unique
- **softwareBackground**: Optional, must be valid enum value if provided
- **hardwareBackground**: Optional, must be valid enum value if provided
- **programmingLanguages**: Optional, array of valid language strings if provided
- **roboticsExperience**: Optional, must be valid enum value if provided
- **aiMlExperience**: Optional, must be valid enum value if provided
- **hasRosExperience**: Optional, boolean value
- **hasGpuAccess**: Optional, boolean value
- **learningGoals**: Optional, max 1000 characters

## Authentication State Entity

### Authentication State
- **isAuthenticated**: boolean (whether user is currently authenticated)
- **user**: UserProfile | null (user profile data if authenticated, null otherwise)
- **loading**: boolean (whether authentication state is being determined)
- **error**: string | null (error message if authentication failed)

### Validation Rules
- **isAuthenticated**: Always present, boolean value
- **user**: UserProfile object if authenticated, null if not
- **loading**: Always present, boolean value
- **error**: String if error occurred, null if no error

## Form State Entities

### Signup Form State
- **name**: string (user's name)
- **email**: string (user's email)
- **password**: string (user's password)
- **softwareBackground**: string (selected software background)
- **hardwareBackground**: string (selected hardware background)
- **programmingLanguages**: string[] (selected programming languages)
- **roboticsExperience**: string (selected robotics experience)
- **aiMlExperience**: string (selected AI/ML experience)
- **hasRosExperience**: boolean (ROS experience toggle)
- **hasGpuAccess**: boolean (GPU access toggle)
- **learningGoals**: string (learning goals text)
- **errors**: object (field-specific error messages)
- **loading**: boolean (form submission state)

### Login Form State
- **email**: string (user's email)
- **password**: string (user's password)
- **showPassword**: boolean (whether to show password in plain text)
- **errors**: object (field-specific error messages)
- **loading**: boolean (form submission state)

## Modal State Entity

### Modal State
- **showSignupModal**: boolean (whether signup modal is visible)
- **showLoginModal**: boolean (whether login modal is visible)
- **showProfileDropdown**: boolean (whether profile dropdown is visible)

### Validation Rules
- Only one modal type should be open at a time (signup or login)
- Profile dropdown is independent of modals
- All values are boolean

## Session Management Entity

### Session Data
- **session**: object (session data from Better Auth)
- **expiresAt**: Date (timestamp when session expires)
- **lastActivity**: Date (timestamp of last user activity)

### Validation Rules
- **session**: Valid session object from Better Auth
- **expiresAt**: Future date/time, automatically set based on timeout configuration
- **lastActivity**: Updated on each user interaction

## UI State Entities

### Loading States
- **isSubmitting**: boolean (form submission in progress)
- **isValidating**: boolean (form validation in progress)
- **isFetching**: boolean (data fetching in progress)

### Animation States
- **modalAnimation**: string (current modal animation state)
- **dropdownAnimation**: string (current dropdown animation state)

## State Transitions

### Authentication Flow
1. **Unauthenticated** → **Login/Signup** → **Authenticated** → **Logout** → **Unauthenticated**
2. **Authenticated** → **Session Timeout** → **Unauthenticated**

### Modal Flow
1. **Closed** → **Opening** → **Open** → **Closing** → **Closed**

## Relationships

### User Profile to Authentication State
- Authentication State has 0..1 User Profile (when authenticated)
- User Profile does not directly reference Authentication State

### Form States to Authentication State
- Signup Form State and Login Form State are independent of Authentication State
- Form submission may affect Authentication State