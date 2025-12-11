# Quickstart Guide: Authentication System Frontend Redesign

## Overview
This guide provides instructions for setting up, running, and testing the authentication system frontend redesign for the Docusaurus-based robotics textbook website.

## Prerequisites

### System Requirements
- Node.js v18 or higher
- npm or yarn package manager
- Git for version control
- Modern web browser (Chrome, Firefox, Safari, Edge)

### Project Dependencies
- Docusaurus 2.x framework
- Better Auth client library
- React v18+
- TypeScript (optional but recommended)

## Setup Instructions

### 1. Clone and Navigate to Project
```bash
cd physical-ai-robotics-textbook/docusaurus
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Verify Backend Connection
Ensure the Better Auth backend is running and accessible at the configured endpoint.

## Development Workflow

### 1. Starting the Development Server
```bash
npm run dev
# or
yarn dev
```

### 2. Running Tests
```bash
npm run test
# or
yarn test
```

### 3. Building for Production
```bash
npm run build
# or
yarn build
```

## Component Integration

### 1. AuthButtons Component
The main navbar integration component that shows different UI based on authentication state:

```tsx
import AuthButtons from '@site/src/components/auth/AuthButtons';
import { authClient } from '@site/src/lib/auth-client';

// Usage in navbar
<AuthButtons client={authClient} />
```

### 2. Modal Components
The signup and login modals are managed by the AuthButtons component but can be used independently:

- `SignupModal.tsx` - Handles new user registration with background profile
- `LoginModal.tsx` - Handles existing user authentication
- `ProfileDropdown.tsx` - Shows user profile information when authenticated

## Environment Configuration

### Required Environment Variables
```bash
# In your .env file
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:5000  # Or your production backend URL
```

### Auth Client Configuration
The `auth-client.ts` file is pre-configured to work with Better Auth:

```ts
export const authClient = createAuthClient({
  baseURL: getBaseURL(),
  fetchOptions: {
    credentials: 'include', // Important for cookie handling
  },
});
```

## Testing the Authentication Flow

### 1. Unauthenticated State
- Visit any page on the website
- Observe "Login" and "Sign Up" buttons in the navbar
- Click "Sign Up" to open the signup modal
- Click "Login" to open the login modal

### 2. Registration Flow
- Open the signup modal
- Fill in all required fields (name, email, password)
- Provide background information (optional but recommended)
- Submit the form
- Verify account creation and automatic login

### 3. Login Flow
- Open the login modal
- Enter registered email and password
- Submit the form
- Verify successful authentication and navbar update

### 4. Profile Access
- After authentication, click the profile avatar in the navbar
- Verify the profile dropdown shows user information
- Test the logout functionality

### 5. Session Management
- Verify automatic session timeout after 30 minutes of inactivity
- Test that modals show appropriate error messages for invalid credentials
- Confirm that error messages are generic to prevent information disclosure

## Common Development Tasks

### 1. Modifying Styling
- Update `Auth.module.css` for component-specific styles
- Use CSS Modules to avoid style conflicts
- Follow the orange and white color theme

### 2. Adding Form Fields
- Update the SignupModal component state
- Add validation for new fields
- Update the backend to handle new profile data

### 3. Customizing Animations
- Modify CSS transitions in `Auth.module.css`
- Adjust timing in the animation classes
- Test performance on lower-end devices

## Troubleshooting

### Common Issues

#### 1. Navbar Buttons Not Appearing
- Verify `AuthButtons` component is properly integrated in `Navbar/Content/index.js`
- Check that auth-client is properly configured
- Ensure the component is rendering in the correct location

#### 2. Modal Not Opening
- Verify the modal state management in `AuthButtons.tsx`
- Check for JavaScript errors in the browser console
- Ensure all required dependencies are installed

#### 3. Authentication Not Working
- Verify backend API endpoints are accessible
- Check network requests in browser developer tools
- Confirm auth-client configuration matches backend

#### 4. Styling Issues
- Check for CSS conflicts with Docusaurus default styles
- Verify CSS Modules are properly scoped
- Ensure responsive design works across devices

## Security Best Practices

### 1. Password Requirements
- Enforce minimum 8 characters with uppercase, lowercase, number, and special character
- Implement client-side validation with backend enforcement
- Never log passwords or sensitive credentials

### 2. Error Handling
- Display generic error messages to users
- Log detailed errors server-side for debugging
- Implement rate limiting to prevent brute force attacks

### 3. Session Management
- Implement automatic timeout after 30 minutes of inactivity
- Use secure cookies with appropriate flags
- Validate sessions on sensitive operations

## Performance Optimization

### 1. Component Loading
- Implement lazy loading for modal components
- Optimize re-renders using React.memo where appropriate
- Minimize bundle size with code splitting

### 2. Animation Performance
- Use CSS transforms and opacity for animations
- Implement hardware acceleration for smooth transitions
- Test performance on lower-end devices

### 3. Network Requests
- Optimize API calls with proper caching
- Implement loading states for better UX
- Handle network errors gracefully