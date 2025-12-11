# API Contracts: Authentication System Frontend

## Overview
This document defines the API contracts between the frontend authentication components and the backend Better Auth system for the Docusaurus-based robotics textbook website.

## Session Management Contracts

### GET /api/auth/get-session
**Purpose**: Retrieve current user session information
**Frontend Trigger**: Component mount, periodic refresh, after auth actions
**Request**:
- Method: GET
- Headers: Cookie with auth session
- Body: None

**Response**:
- Success (200):
  ```json
  {
    "user": {
      "id": "string",
      "name": "string",
      "email": "string",
      "softwareBackground": "string (optional)",
      "hardwareBackground": "string (optional)",
      "programmingLanguages": "string[] (optional)",
      "roboticsExperience": "string (optional)",
      "aiMlExperience": "string (optional)",
      "hasRosExperience": "boolean (optional)",
      "hasGpuAccess": "boolean (optional)",
      "learningGoals": "string (optional)"
    }
  }
  ```
- Unauthorized (401): Empty response or error object

**Frontend Handling**: Update authentication state, show/hide UI elements based on session status

### POST /api/auth/sign-in/email
**Purpose**: Authenticate user with email and password
**Frontend Trigger**: Login form submission
**Request**:
- Method: POST
- Headers: Content-Type: application/json
- Body:
  ```json
  {
    "email": "string (required)",
    "password": "string (required)"
  }
  ```

**Response**:
- Success (200): Session cookie set, returns user object
- Error (400/401):
  ```json
  {
    "error": "string (generic error message)"
  }
  ```

**Frontend Handling**: Show success/error feedback, update auth state, close modal on success

### POST /api/auth/sign-up/email
**Purpose**: Register new user with credentials and profile data
**Frontend Trigger**: Signup form submission
**Request**:
- Method: POST
- Headers: Content-Type: application/json
- Body:
  ```json
  {
    "email": "string (required)",
    "password": "string (required)",
    "name": "string (required)",
    "profile": {
      "softwareBackground": "string (optional)",
      "hardwareBackground": "string (optional)",
      "programmingLanguages": "string[] (optional)",
      "roboticsExperience": "string (optional)",
      "aiMlExperience": "string (optional)",
      "hasRosExperience": "boolean (optional)",
      "hasGpuAccess": "boolean (optional)",
      "learningGoals": "string (optional)"
    }
  }
  ```

**Response**:
- Success (200): Session cookie set, returns user object
- Error (400):
  ```json
  {
    "error": "string (generic error message)",
    "fieldErrors": {
      "email": "string (optional)",
      "password": "string (optional)",
      "name": "string (optional)"
    }
  }
  ```

**Frontend Handling**: Show success/error feedback, update auth state, close modal on success

### POST /api/auth/sign-out
**Purpose**: End current user session
**Frontend Trigger**: Logout button click
**Request**:
- Method: POST
- Headers: Cookie with auth session
- Body: None

**Response**:
- Success (200): Session cookie cleared
- Error (400/500): Error object

**Frontend Handling**: Update auth state to unauthenticated, show login/signup buttons, close profile dropdown

### POST /api/auth/verify-email
**Purpose**: Verify user email address
**Frontend Trigger**: Email verification flow
**Request**:
- Method: POST
- Headers: Content-Type: application/json
- Body:
  ```json
  {
    "token": "string (verification token)"
  }
  ```

**Response**:
- Success (200): Email verified status
- Error (400): Error object

**Frontend Handling**: Show verification status to user

### POST /api/auth/forgot-password
**Purpose**: Initiate password reset process
**Frontend Trigger**: Forgot password flow
**Request**:
- Method: POST
- Headers: Content-Type: application/json
- Body:
  ```json
  {
    "email": "string (user's email)"
  }
  ```

**Response**:
- Success (200): Password reset initiated
- Error (400): Error object

**Frontend Handling**: Show success message to user

### POST /api/auth/reset-password
**Purpose**: Reset user password with token
**Frontend Trigger**: Password reset flow
**Request**:
- Method: POST
- Headers: Content-Type: application/json
- Body:
  ```json
  {
    "token": "string (reset token)",
    "newPassword": "string (new password)"
  }
  ```

**Response**:
- Success (200): Password reset complete
- Error (400): Error object

**Frontend Handling**: Show completion status to user

## Component State Contracts

### AuthButtons Component
**Props**:
- client: Better Auth client instance (required)

**State Transitions**:
1. Loading → Shows skeleton while checking session
2. Unauthenticated → Shows Login/Signup buttons
3. Authenticated → Shows profile avatar

**Events Emitted**:
- onAuthChange: Triggered when authentication state changes

### SignupModal Component
**Props**:
- isOpen: boolean (required)
- onClose: function (required)
- onSignupSuccess: function (required)

**Validation Requirements**:
- Name: 1-100 characters
- Email: Valid email format
- Password: Minimum 8 characters with uppercase, lowercase, number, special character

**Success Flow**:
1. Form validation passes
2. API call to sign-up endpoint
3. Session established
4. onSignupSuccess callback executed
5. Modal closes

### LoginModal Component
**Props**:
- isOpen: boolean (required)
- onClose: function (required)
- onLoginSuccess: function (required)

**Validation Requirements**:
- Email: Valid email format
- Password: Minimum 1 character (validation handled by backend)

**Success Flow**:
1. Form validation passes
2. API call to sign-in endpoint
3. Session established
4. onLoginSuccess callback executed
5. Modal closes

### ProfileDropdown Component
**Props**:
- session: User session object (required)
- onLogout: function (required)

**Display Requirements**:
- Shows user name and email
- Shows all available profile information
- Shows logout button

**Interaction Flow**:
1. Click avatar to open dropdown
2. Click logout button to trigger onLogout
3. onLogout calls sign-out API
4. Authentication state updates

## Error Handling Contracts

### Network Error Handling
- Timeout: 30 seconds for all auth requests
- Retry: 1 retry on network errors
- User Feedback: Generic error messages displayed

### Validation Error Handling
- Field-level validation with immediate feedback
- Form-level validation before submission
- Error messages follow security requirements (no detailed disclosure)

## Security Contracts

### Data Transmission
- All auth data transmitted over HTTPS
- Passwords never logged or stored client-side
- Session tokens handled by auth library

### Error Message Security
- Generic error messages shown to users
- Detailed logs maintained server-side
- No information disclosure about account existence

### Session Management
- Automatic timeout after 30 minutes of inactivity
- Secure cookie flags used
- Session state validated on sensitive operations