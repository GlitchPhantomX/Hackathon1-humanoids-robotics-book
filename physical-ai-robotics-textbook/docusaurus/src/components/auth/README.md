# Authentication Components

This directory contains the authentication components for the Docusaurus application, providing login, signup, and user management functionality.

## Components

### AuthButtons
The main authentication component that displays login/signup buttons when the user is not authenticated, or a user profile dropdown when authenticated.

**Props:**
- `onAuthChange`: Optional callback function called when authentication state changes

### LoginModal
A modal component for user login with email and password.

**Props:**
- `isOpen`: Boolean to control modal visibility
- `onClose`: Function to close the modal
- `onLoginSuccess`: Function called when login is successful
- `onSwitchToSignup`: Function to switch to signup form

### SignupModal
A two-step modal component for user registration with account information and personalization details.

**Props:**
- `isOpen`: Boolean to control modal visibility
- `onClose`: Function to close the modal
- `onSignupSuccess`: Function called when signup is successful

## Features

- **Responsive Design**: All components are fully responsive and mobile-friendly
- **Accessibility**: Full keyboard navigation, screen reader support, and ARIA attributes
- **Security**: Secure authentication with Better Auth
- **Performance**: Optimized with React.memo and useCallback
- **Error Handling**: User-friendly error messages and validation
- **Loading States**: Visual feedback during authentication processes
- **Type Safety**: Full TypeScript support with comprehensive interfaces

## Accessibility

The auth components include comprehensive accessibility features:

- Proper ARIA attributes (roles, labels, descriptions)
- Keyboard navigation support (Tab, Enter, Space, Escape)
- Screen reader announcements for important state changes
- Focus management and trapping
- Reduced motion support
- High contrast mode support
- Proper semantic HTML structure

## Styling

The components use CSS Modules for styling with the following features:

- Responsive breakpoints for mobile and desktop
- CSS animations and transitions
- Focus states for keyboard navigation
- Visual feedback for interactive elements
- Support for reduced motion and high contrast modes

## API Integration

The components integrate with the backend authentication API:

- Login and signup endpoints
- User profile management
- Session management
- Error handling and validation

## Error Handling

Comprehensive error handling for various scenarios:

- Invalid email/password
- Account already exists
- Network errors
- Validation errors
- User-friendly error messages