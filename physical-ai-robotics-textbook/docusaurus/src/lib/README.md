# Authentication Client

This file contains the authentication client configuration and helper functions for the frontend application.

## Configuration

The auth client is configured to work with the Better Auth backend:

```typescript
export const authClient = createAuthClient({
  baseURL: process.env.NODE_ENV === 'production'
    ? (process.env.NEXT_PUBLIC_BETTER_AUTH_URL || 'https://your-domain.com')
    : 'http://localhost:5000',
  fetchOptions: {
    credentials: 'include', // Important for cookie handling
  },
});
```

## Available Functions

### Core Authentication
- `signIn`: Sign in with email and password
- `signOut`: Sign out the current user

### Helper Functions

#### `announceToScreenReader(message: string)`
Creates an accessible announcement for screen readers without cluttering the UI.

```typescript
announceToScreenReader('Login successful. Welcome back!');
```

#### `focusElement(elementId: string)`
Safely focuses on an element, making it focusable if necessary.

```typescript
focusElement('login-email-input');
```

## Environment Variables

- `NEXT_PUBLIC_BETTER_AUTH_URL`: The base URL for the authentication API in production

## Security Considerations

- Credentials are included in requests to support cookie-based authentication
- The client is designed to work with CSRF protection
- All authentication requests are made over HTTPS in production

## Error Handling

The auth client handles various authentication errors:

- Network errors
- Invalid credentials
- Account verification requirements
- Rate limiting