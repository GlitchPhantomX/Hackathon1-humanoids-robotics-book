# API Contracts: Better Auth Implementation

## 1. Authentication Endpoints

### 1.1 User Registration
**Endpoint:** `POST /api/auth/sign-up/email`

**Request:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123",
  "name": "John Doe",
  "softwareBackground": "intermediate",
  "hardwareBackground": "basic",
  "programmingLanguages": ["Python", "C++"],
  "roboticsExperience": "academic",
  "aiMlExperience": "intermediate",
  "hasRosExperience": true,
  "hasGpuAccess": false,
  "learningGoals": "Learn to build autonomous robots"
}
```

**Request Headers:**
- `Content-Type: application/json`

**Response (Success):**
```json
{
  "user": {
    "id": "user_abc123",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": false,
    "image": null,
    "createdAt": "2025-12-08T10:00:00.000Z",
    "updatedAt": "2025-12-08T10:00:00.000Z",
    "softwareBackground": "intermediate",
    "hardwareBackground": "basic",
    "programmingLanguages": ["Python", "C++"],
    "roboticsExperience": "academic",
    "aiMlExperience": "intermediate",
    "hasRosExperience": true,
    "hasGpuAccess": false,
    "learningGoals": "Learn to build autonomous robots"
  },
  "session": {
    "token": "sess_xyz789",
    "expiresAt": "2025-12-15T10:00:00.000Z"
  }
}
```

**Response (Error):**
```json
{
  "error": {
    "message": "Email already exists",
    "code": "EMAIL_EXISTS"
  }
}
```

**Validation Rules:**
- Email must be valid format and unique
- Password must be at least 8 characters
- Name must be at least 2 characters
- All required custom fields must be provided
- Enum values must match defined options

### 1.2 User Login
**Endpoint:** `POST /api/auth/sign-in/email`

**Request:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123"
}
```

**Request Headers:**
- `Content-Type: application/json`

**Response (Success):**
```json
{
  "user": {
    "id": "user_abc123",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": false,
    "image": null,
    "createdAt": "2025-12-08T10:00:00.000Z",
    "updatedAt": "2025-12-08T10:00:00.000Z",
    "softwareBackground": "intermediate",
    "hardwareBackground": "basic",
    "roboticsExperience": "academic",
    "aiMlExperience": "intermediate",
    "hasRosExperience": true,
    "hasGpuAccess": false,
    "learningGoals": "Learn to build autonomous robots"
  },
  "session": {
    "token": "sess_xyz789",
    "expiresAt": "2025-12-15T10:00:00.000Z"
  }
}
```

**Response (Error):**
```json
{
  "error": {
    "message": "Invalid email or password",
    "code": "INVALID_CREDENTIALS"
  }
}
```

### 1.3 User Logout
**Endpoint:** `POST /api/auth/sign-out`

**Request Headers:**
- `Cookie: auth_session=...` (session cookie)

**Response (Success):**
```json
{
  "success": true
}
```

**Response (Error):**
```json
{
  "error": {
    "message": "No active session found",
    "code": "NO_SESSION"
  }
}
```

### 1.4 Get Session
**Endpoint:** `GET /api/auth/session`

**Request Headers:**
- `Cookie: auth_session=...` (session cookie)

**Response (Success):**
```json
{
  "user": {
    "id": "user_abc123",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": false,
    "image": null,
    "createdAt": "2025-12-08T10:00:00.000Z",
    "updatedAt": "2025-12-08T10:00:00.000Z",
    "softwareBackground": "intermediate",
    "hardwareBackground": "basic",
    "roboticsExperience": "academic",
    "aiMlExperience": "intermediate",
    "hasRosExperience": true,
    "hasGpuAccess": false,
    "learningGoals": "Learn to build autonomous robots"
  },
  "session": {
    "token": "sess_xyz789",
    "expiresAt": "2025-12-15T10:00:00.000Z"
  }
}
```

**Response (Error):**
```json
{
  "error": {
    "message": "No active session found",
    "code": "NO_SESSION"
  }
}
```

## 2. Custom Endpoints

### 2.1 Get User Profile
**Endpoint:** `GET /api/user/profile`

**Request Headers:**
- `Cookie: auth_session=...` (session cookie)

**Response (Success):**
```json
{
  "user": {
    "id": "user_abc123",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": false,
    "image": null,
    "createdAt": "2025-12-08T10:00:00.000Z",
    "updatedAt": "2025-12-08T10:00:00.000Z",
    "softwareBackground": "intermediate",
    "hardwareBackground": "basic",
    "programmingLanguages": ["Python", "C++"],
    "roboticsExperience": "academic",
    "aiMlExperience": "intermediate",
    "hasRosExperience": true,
    "hasGpuAccess": false,
    "learningGoals": "Learn to build autonomous robots"
  }
}
```

### 2.2 Update User Profile
**Endpoint:** `PATCH /api/user/profile`

**Request:**
```json
{
  "name": "John Smith",
  "learningGoals": "Learn to build humanoid robots",
  "hasGpuAccess": true
}
```

**Request Headers:**
- `Content-Type: application/json`
- `Cookie: auth_session=...` (session cookie)

**Response (Success):**
```json
{
  "user": {
    "id": "user_abc123",
    "email": "user@example.com",
    "name": "John Smith",
    "emailVerified": false,
    "image": null,
    "createdAt": "2025-12-08T10:00:00.000Z",
    "updatedAt": "2025-12-08T11:00:00.000Z",
    "softwareBackground": "intermediate",
    "hardwareBackground": "basic",
    "programmingLanguages": ["Python", "C++"],
    "roboticsExperience": "academic",
    "aiMlExperience": "intermediate",
    "hasRosExperience": true,
    "hasGpuAccess": true,
    "learningGoals": "Learn to build humanoid robots"
  }
}
```

## 3. Error Handling

### 3.1 Error Response Format
All error responses follow the same format:
```json
{
  "error": {
    "message": "Human-readable error message",
    "code": "ERROR_CODE",
    "status": 400
  }
}
```

### 3.2 Common Error Codes
- `EMAIL_EXISTS`: Email already registered
- `INVALID_CREDENTIALS`: Incorrect email or password
- `NO_SESSION`: No active session found
- `INVALID_EMAIL_FORMAT`: Email format is invalid
- `WEAK_PASSWORD`: Password doesn't meet requirements
- `MISSING_REQUIRED_FIELD`: Required field not provided
- `INVALID_ENUM_VALUE`: Enum value not in allowed list
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `INTERNAL_ERROR`: Server error

### 3.3 HTTP Status Codes
- `200`: Success
- `201`: Created
- `400`: Bad request (validation errors)
- `401`: Unauthorized (invalid session)
- `403`: Forbidden
- `404`: Not found
- `429`: Rate limited
- `500`: Internal server error

## 4. Security Headers

### 4.1 Required Headers
- `Content-Type: application/json` for JSON requests
- `Cookie` for session authentication
- `Authorization` for token-based auth (if implemented)

### 4.2 Security Measures
- All auth cookies are HTTP-only and secure
- CSRF protection enabled by default
- Rate limiting on auth endpoints
- Input validation on all fields

## 5. Session Management

### 5.1 Session Configuration
- **Duration:** 7 days (604,800 seconds)
- **Update Age:** 1 day (86,400 seconds)
- **Cookie Cache:** 5 minutes (300 seconds)
- **Storage:** Database with token-based sessions

### 5.2 Session Lifecycle
1. Created on successful login/registration
2. Validated on each protected request
3. Extended when update age is reached
4. Destroyed on logout or expiration
5. Renewed on activity within refresh window

## 6. Rate Limiting

### 6.1 Rate Limits
- Login attempts: 10 per minute per IP
- Registration: 5 per minute per IP
- Password reset: 3 per hour per email
- Session validation: 100 per minute per user

### 6.2 Response Headers
Rate limited responses include:
- `Retry-After`: Seconds to wait before next request
- `X-RateLimit-Remaining`: Requests remaining in current window
- `X-RateLimit-Reset`: Unix timestamp when window resets