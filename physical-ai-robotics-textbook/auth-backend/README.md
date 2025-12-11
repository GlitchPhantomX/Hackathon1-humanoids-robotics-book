# Authentication Backend

This is the authentication backend service built with Express and Better Auth, providing secure authentication for the Docusaurus application.

## Features

### Security
- Helmet security headers for protection against common web vulnerabilities
- Rate limiting to prevent abuse and brute force attacks
- CORS configuration for secure cross-origin requests
- Content Security Policy (CSP) for XSS protection
- HSTS for HTTPS enforcement in production

### Authentication
- Email/password authentication
- Session management
- User profile management
- Secure credential handling

### Middleware
- Request logging with different log levels
- Error logging for debugging
- Environment-specific configurations (dev/prod)

## API Endpoints

### Authentication Endpoints
- `GET/POST/PUT/DELETE /api/auth/*` - Better Auth routes
- `GET /api/user/profile` - Get current user profile
- `PATCH /api/user/profile` - Update user profile with background information
- `PUT /api/user/profile` - Alternative endpoint to update user profile

### Security Configuration
- Rate limiting: 100 requests per 15 minutes per IP
- Content Security Policy with appropriate directives
- XSS protection headers
- Clickjacking protection

## Environment Variables

- `NODE_ENV`: Environment mode (development/production)
- `PORT`: Server port (default: 5000)
- `FRONTEND_URL`: Frontend application URL for CORS

## Configuration

### Development vs Production
- Development: More permissive CORS settings
- Production: Strict security headers, HSTS, trust proxy settings

### Logging
- Different log levels (info, warn, error, debug)
- Request logging with method, URL, status code, duration
- Error logging with stack traces
- User agent and IP tracking

## Dependencies

- `better-auth`: Authentication library
- `express`: Web framework
- `cors`: Cross-origin resource sharing
- `helmet`: Security headers
- `express-rate-limit`: Rate limiting
- `dotenv`: Environment variables

## Error Handling

- Proper error responses with appropriate status codes
- Detailed error logging for debugging
- User-friendly error messages
- Validation error handling