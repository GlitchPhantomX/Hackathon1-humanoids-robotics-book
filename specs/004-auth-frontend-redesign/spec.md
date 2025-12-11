# Feature Specification: Authentication System Frontend Redesign

**Feature Branch**: `004-auth-frontend-redesign`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Authentication System Implementation - Claude Code Prompt

## Project Overview
I'm building a Docusaurus-based robotics textbook website with full authentication. The backend (Better Auth + PostgreSQL) is *fully functional*. I need help implementing a professional, polished frontend UI.

---

## 🎯 Requirements

### 1. Navbar Integration (Top Priority)
*Location:* Docusaurus navbar, right side, after GitHub link and language selector

*Components Needed:*
- When *logged out*: Show two buttons
  - Login button
  - Sign Up button
- When *logged in*: Show single element
  - Profile avatar (circular, shows user's first letter or name)

*Visual Style:*
- Match Docusaurus navbar styling
- Smooth transitions between logged in/out states
- Orange and white theme colors
- Professional, clean appearance

---

### 2. Sign Up Modal
*Trigger:* Click 'Sign Up' button in navbar

*Behavior:*
- Opens as *centered modal* overlay on screen
- *Animated entrance* (fade in + scale up, 300ms)
- *Closes when:*
  - User clicks outside modal (backdrop click)
  - User presses Escape key
  - Successful signup completes

*Design:*
- *Theme:* Orange (#FF6B35 or similar) and white
- *Professional styling:*
  - Clean, modern card design
  - Proper spacing and padding
  - Smooth shadows
  - Rounded corners
- *Backdrop:* Semi-transparent dark overlay (rgba(0,0,0,0.5))

*Form Fields:*
All fields with professional styling (floating labels, focus states, validation):

1. *Basic Info:*
   - Name (required)
   - Email (required, validated)
   - Password (required, min 8 chars, show/hide toggle)

2. *Background Profile:*
   - Software Background (dropdown: beginner, intermediate, advanced, expert)
   - Hardware Background (dropdown: none, basic, intermediate, advanced)
   - Programming Languages (multi-select checkboxes: Python, JavaScript, C++, Java, ROS, etc.)
   - Robotics Experience (dropdown: none, hobbyist, academic, professional)
   - AI/ML Experience (dropdown: none, basic, intermediate, advanced)
   - Has ROS Experience? (toggle/checkbox)
   - Has GPU Access? (toggle/checkbox)
   - Learning Goals (textarea)

3. *Submit Button:*
   - Orange background
   - White text
   - Loading state with spinner
   - Disabled during submission

---

### 3. Login Modal
*Trigger:* Click 'Login' button in navbar

*Same behavior as Sign Up modal:*
- Centered overlay
- Animated entrance
- Closes on backdrop click or Escape
- Orange & white theme

*Form Fields:*
- Email (required)
- Password (required, show/hide toggle)
- 'Sign Up' link at bottom (switches to signup modal)
- Submit button with loading state

---

### 4. Profile Dropdown
*Trigger:* Click profile avatar when logged in

*Behavior:*
- Dropdown appears *below* the profile avatar
- Properly positioned (right-aligned with avatar)
- High z-index (appears above all content)
- Closes when clicking outside

*Content Structure:*

┌─────────────────────────────────┐
│  [Name]                         │
│  user@email.com                 │
├─────────────────────────────────┤
│  Background Profile             │
│  ─────────────────────          │
│  Software: intermediate         │
│  Hardware: basic                │
│  Languages: Python, JavaScript  │
│  Robotics: hobbyist             │
│  AI/ML: basic                   │
│  [✓ ROS] [✓ GPU]                │
│  Goals: Learning robotics...    │
├─────────────────────────────────┤
│  [Logout Button]                │
└─────────────────────────────────┘


*Styling:*
- Clean card design
- Orange accents
- Professional spacing
- Scrollable if content too long
- Width: 320-400px

---

## 🏗 Technical Details

### Tech Stack
- *Frontend:* Docusaurus (React)
- *Backend:* Better Auth (already working)
- *Database:* PostgreSQL with Drizzle ORM
- *Auth Client:* @site/src/lib/auth-client.ts

### Project Structure

docusaurus/
├── src/
│   ├── components/
│   │   └── auth/
│   │       ├── AuthButtons.tsx (needs major refactor)
│   │       ├── SignupModal.tsx (needs redesign)
│   │       ├── LoginModal.tsx (needs redesign)
│   │       ├── ProfileDropdown.tsx (needs creation)
│   │       └── Auth.module.css (needs complete overhaul)
│   ├── lib/
│   │   └── auth-client.ts (working, don't touch)
│   └── theme/
│       ├── Root.tsx (clean up)
│       └── Navbar/
│           └── Content/
│               └── index.js (integrate AuthButtons here)


### Backend API (Already Working)
- POST /api/auth/sign-up/email - Creates user
- POST /api/auth/sign-in/email - Authenticates user
- GET /api/auth/get-session - Gets current session
- POST /api/auth/sign-out - Logs out user

### Session Data Structure
typescript
{
  user: {
    id: string;
    name: string;
    email: string;
    softwareBackground?: 'beginner' | 'intermediate' | 'advanced' | 'expert';
    hardwareBackground?: 'none' | 'basic' | 'intermediate' | 'advanced';
    programmingLanguages?: string[]; // ['Python', 'JavaScript', etc.]
    roboticsExperience?: 'none' | 'hobbyist' | 'academic' | 'professional';
    aiMlExperience?: 'none' | 'basic' | 'intermediate' | 'advanced';
    hasRosExperience?: boolean;
    hasGpuAccess?: boolean;
    learningGoals?: string;
  }
}


---

## 🎨 Design Requirements

### Color Palette
- *Primary:* Orange (#FF6B35 or similar professional orange)
- *Secondary:* White (#FFFFFF)
- *Text:* Dark gray (#333333)
- *Borders:* Light gray (#E0E0E0)
- *Hover:* Darker orange
- *Background overlay:* rgba(0, 0, 0, 0.5)

### Animations
- Modal entrance: Fade + scale (0.95 → 1.0)
- Duration: 300ms
- Easing: ease-out
- Button hover: Subtle scale/color transition
- Dropdown: Smooth slide down

### Responsive Design
- Desktop: Full features
- Tablet: Adjust modal width
- Mobile: Full-width modals with proper padding

---

## ✅ What I Need from You

### 1. Complete Component Files
*Create/Update these files with full implementation:*

- src/components/auth/AuthButtons.tsx
  - Navbar integration component
  - Shows Login/Signup when logged out
  - Shows Profile avatar when logged in
  - Manages modal states

- src/components/auth/SignupModal.tsx
  - Professional centered modal
  - All form fields with validation
  - Orange & white theme
  - Animated entrance/exit
  - Backdrop click handling

- src/components/auth/LoginModal.tsx
  - Similar to signup but simpler
  - Email + password only
  - Link to switch to signup

- src/components/auth/ProfileDropdown.tsx
  - Dropdown below avatar
  - Shows all user background data
  - Logout button
  - Proper positioning & z-index

- src/components/auth/Auth.module.css
  - All styling for above components
  - Professional, polished look
  - Orange & white theme
  - Animations included

### 2. Navbar Integration
*Update:* src/theme/Navbar/Content/index.js
- Properly integrate AuthButtons component
- Ensure it appears after language selector
- No layout breaking

### 3. Root Component Cleanup
*Update:* src/theme/Root.tsx
- Remove any AuthButtons code
- Keep it clean (only ChatWidget if needed)

### 4. Custom CSS
*Update:* src/css/custom.css
- Global styles for modals
- Z-index management
- Smooth transitions

---

## 📋 Implementation Checklist

Create components in this order:
1. ✅ AuthButtons.tsx (navbar component)
2. ✅ SignupModal.tsx (with all fields)
3. ✅ LoginModal.tsx
4. ✅ ProfileDropdown.tsx
5. ✅ Auth.module.css (complete styling)
6. ✅ Integrate into Navbar/Content/index.js
7. ✅ Test all flows
8. ✅ Polish animations and styling

---

## 🔧 Key Technical Requirements

### State Management
- Use React hooks (useState, useEffect)
- Fetch session on mount
- Re-fetch after login/signup/logout
- Handle loading states

### Error Handling
- Show validation errors inline
- Network error messages
- Form submission feedback

### Accessibility
- Proper ARIA labels
- Keyboard navigation (Tab, Escape)
- Focus management in modals
- Screen reader friendly

### Performance
- Lazy load modals if possible
- Optimize re-renders
- Smooth 60fps animations

---

## 🚫 What NOT to Change

*Leave these files untouched (they're working):*
- src/lib/auth-client.ts
- Backend auth configuration
- Database schema
- Any backend files

---

## 📝 Final Notes

- *Quality over speed* - Make it professional and polished
- *Consistent styling* - Follow Docusaurus theme
- *User experience* - Smooth, intuitive interactions
- *Mobile-first* - Works perfectly on all devices
- *Test thoroughly* - All login/signup/logout flows work

Please provide complete, production-ready code for all components listed above."

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Access Authentication Features from Navbar (Priority: P1)

As a visitor to the robotics textbook website, I want to see clear login and signup options in the navbar so I can create an account or sign in to access personalized content.

**Why this priority**: This is the entry point for all authentication flows and provides immediate access to the core functionality that enables personalized learning experiences.

**Independent Test**: Can be fully tested by visiting the website and verifying that login/signup buttons appear in the navbar, and clicking them triggers the appropriate modals.

**Acceptance Scenarios**:

1. **Given** I am on any page of the website, **When** I look at the navbar, **Then** I see login and signup buttons on the right side
2. **Given** I am on the website and not logged in, **When** I click the signup button, **Then** a professional signup modal appears with animated entrance

---

### User Story 2 - Create Account with Background Profile (Priority: P2)

As a new user, I want to create an account and provide my background information so the system can tailor content to my skill level and interests.

**Why this priority**: This enables the core value proposition of personalized learning experiences based on user background and goals.

**Independent Test**: Can be fully tested by opening the signup modal and completing the registration process with all background information.

**Acceptance Scenarios**:

1. **Given** I have opened the signup modal, **When** I fill in all required fields and submit, **Then** my account is created and I am logged in
2. **Given** I am filling out the signup form, **When** I enter invalid information, **Then** I see clear validation errors
3. **Given** I am on the signup form, **When** I select my background information, **Then** this data is saved with my profile

---

### User Story 3 - Sign In and Access Profile (Priority: P2)

As an existing user, I want to sign in with my credentials and access my profile information so I can manage my account and see my background details.

**Why this priority**: This enables returning users to access their personalized experience and continue their learning journey.

**Independent Test**: Can be fully tested by opening the login modal, signing in, and accessing the profile dropdown.

**Acceptance Scenarios**:

1. **Given** I have opened the login modal, **When** I enter valid credentials and submit, **Then** I am logged in and the navbar shows my profile avatar
2. **Given** I am logged in, **When** I click my profile avatar, **Then** a dropdown appears showing my background information
3. **Given** I am logged in, **When** I click the logout button in the profile dropdown, **Then** I am logged out and the navbar shows login/signup buttons again

---

### User Story 4 - Professional UI with Orange Theme (Priority: P3)

As a user, I want a professional, polished UI with consistent orange theming so the authentication experience feels cohesive with the website design.

**Why this priority**: This enhances user experience and brand consistency, making the authentication process feel integrated and professional.

**Independent Test**: Can be fully tested by examining all authentication UI components for consistent styling, animations, and responsive design.

**Acceptance Scenarios**:

1. **Given** I am viewing any authentication component, **When** I examine the styling, **Then** it follows the orange and white theme consistently
2. **Given** I am using the website on different devices, **When** I interact with authentication components, **Then** they are responsive and properly styled

---

### Edge Cases

- What happens when a user closes the modal during signup without completing it?
- How does the system handle network errors during authentication requests?
- What occurs when a user's session expires while they have the profile dropdown open?
- How does the system handle invalid or expired authentication tokens?
- What happens when a user tries to access protected content while not logged in?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST display login and signup buttons in the navbar when user is not authenticated
- **FR-002**: System MUST display a profile avatar in the navbar when user is authenticated
- **FR-003**: Users MUST be able to open a signup modal by clicking the signup button in the navbar
- **FR-004**: System MUST validate all signup form fields before submission and show clear error messages
- **FR-005**: Users MUST be able to provide comprehensive background information during signup including software/hardware experience, programming languages, robotics experience, and learning goals
- **FR-006**: Users MUST be able to open a login modal by clicking the login button in the navbar
- **FR-007**: System MUST authenticate users with email and password credentials
- **FR-008**: System MUST display a profile dropdown when authenticated user clicks their profile avatar
- **FR-009**: Profile dropdown MUST show all user background information in an organized, readable format
- **FR-010**: Users MUST be able to log out by clicking the logout button in the profile dropdown
- **FR-011**: System MUST handle authentication state changes and update the UI accordingly
- **FR-012**: All authentication modals MUST have proper keyboard navigation and accessibility support
- **FR-013**: System MUST follow responsive design principles for all authentication components
- **FR-014**: Authentication components MUST follow the specified orange and white color theme
- **FR-015**: System MUST include smooth animations for modal entrances and state transitions

### Key Entities *(include if feature involves data)*

- **User Profile**: Represents authenticated user with personal information (name, email) and background details (software/hardware background, programming languages, robotics/AI/ML experience, learning goals, ROS/GPU access indicators)
- **Authentication State**: Represents the current authentication status (logged in/out) and associated user data
- **Background Information**: Structured data about user's technical background, experience levels, and learning objectives

## Clarifications

### Session 2025-12-09

- Q: For security purposes, should user sessions have an automatic timeout after a period of inactivity? → A: Yes, timeout after 30 minutes of inactivity
- Q: What specific requirements should passwords meet for security? → A: At least 8 characters with uppercase, lowercase, number, and special character
- Q: How specific should error messages be when authentication fails? → A: Show generic messages to users (e.g., "Invalid credentials") but log details server-side
- Q: Should new user accounts require email verification before full access? → A: Yes, require email verification before full access
- Q: How should password recovery be implemented for users who forget their password? → A: Email-based password reset with temporary token sent to user's email

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can see login/signup buttons in the navbar within 1 second of page load
- **SC-002**: Users can complete the signup process with all background information in under 3 minutes
- **SC-003**: 95% of users successfully authenticate (login or signup) on their first attempt
- **SC-004**: Authentication modals respond to user interactions within 300ms
- **SC-005**: All authentication components are accessible and keyboard navigable
- **SC-006**: Authentication UI appears correctly across desktop, tablet, and mobile devices
- **SC-007**: User background profile information is accurately displayed in the profile dropdown
- **SC-008**: Authentication state changes are reflected in the UI within 500ms of the action
- **SC-009**: Users can successfully log out and return to unauthenticated state with navbar buttons visible
- **SC-010**: User sessions automatically timeout after 30 minutes of inactivity for security
- **SC-011**: Passwords must be at least 8 characters with uppercase, lowercase, number, and special character
- **SC-012**: Error messages shown to users will be generic (e.g., "Invalid credentials") while detailed logs are maintained server-side
- **SC-013**: New user accounts require email verification before full access
- **SC-014**: Password recovery implemented via email with temporary reset tokens
