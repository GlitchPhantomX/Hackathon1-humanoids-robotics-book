# Better Auth
Implementation Constitution
## Professional Authentication System for Physical AI Textbook

---

## PROJECT CONTEXT

You are implementing a complete authentication system for a Docusaurus-based educational textbook on Physical AI & Humanoid Robotics.
 This is for a hackathon where the authentication system will collect user background information during signup to enable future
content personalization.

*Critical Requirements:*
- Must use Better Auth library (https://www.better-auth.com/)
- Must integrate with existing Docusaurus site
- Must collect specific background information during signup
- Frontend components must be in Docusaurus structure
- Backend must be in separate auth-backend folder

---

## DIRECTORY STRUCTURE REQUIREMENTS


C:\new\physical-ai-robotics-textbook\
├── docusaurus\
│   ├── docs\                          (existing - DO NOT MODIFY)
│   ├── src\
│   │   ├── components\
│   │   │   └── auth\                  (CREATE THIS - all auth UI components here)
│   │   ├── lib\                       (CREATE THIS - auth client setup)
│   │   ├── theme\                     (MAY NEED TO SWIZZLE for navbar)
│   │   └── css\
│   ├── static\
│   ├── docusaurus.config.js           (UPDATE - add auth buttons to navbar)
│   └── package.json                   (UPDATE - add dependencies)
│
└── auth-backend\                      (CREATE THIS - entire backend folder)
    ├── src\
    │   ├── index.ts                   (main server file)
    │   ├── lib\
    │   │   └── auth.ts                (Better Auth configuration)
    │   ├── db\
    │   │   ├── schema.ts              (database schema with user fields)
    │   │   └── index.ts               (database connection)
    │   └── routes\                    (optional - if needed)
    ├── package.json
    ├── tsconfig.json
    ├── .env                           (DO NOT COMMIT)
    └── drizzle.config.ts              (for migrations)


---

## PHASE 1: BACKEND SETUP

### Step 1.1: Initialize Backend Project

*Location:* C:\new\physical-ai-robotics-textbook\auth-backend

*Actions:*
1. Create auth-backend folder at project root
2. Initialize npm project with TypeScript support
3. Set "type": "module" in package.json

*Required Dependencies:*
- better-auth (latest version)
- express and @types/express
- cors and @types/cors
- drizzle-orm (Better Auth's recommended ORM)
- postgres (driver for Neon Postgres)
- dotenv
- tsx (for development)
- drizzle-kit (for migrations)

### Step 1.2: Database Schema Design

*File:* auth-backend/src/db/schema.ts

*Required Tables:*

**1. Users Table (users):**
Must include all default Better Auth user fields PLUS these custom fields:

*Authentication Fields (Better Auth defaults):*
- id (text, primary key)
- email (text, unique, not null)
- emailVerified (boolean)
- name (text, not null)
- image (text, nullable)
- createdAt (timestamp)
- updatedAt (timestamp)

*Custom Background Fields (for personalization):*
- softwareBackground (text, not null) - Values: "beginner" | "intermediate" | "advanced" | "expert"
- hardwareBackground (text, not null) - Values: "none" | "basic" | "intermediate" | "advanced"
- programmingLanguages (jsonb, nullable) - Array of strings like ["Python", "C++", "JavaScript"]
- roboticsExperience (text, not null) - Values: "none" | "hobbyist" | "academic" | "professional"
- aiMlExperience (text, not null) - Values: "none" | "basic" | "intermediate" | "advanced"
- hasRosExperience (boolean, not null, default false)
- hasGpuAccess (boolean, not null, default false)
- learningGoals (text, nullable) - Free text field

**2. Sessions Table (sessions):**
- id (text, primary key)
- userId (text, foreign key to users)
- expiresAt (timestamp)
- token (text, unique)
- ipAddress (text, nullable)
- userAgent (text, nullable)

**3. Accounts Table (accounts):**
(for OAuth providers if needed later)
- id (text, primary key)
- userId (text, foreign key to users)
- accountId (text)
- providerId (text)
- accessToken (text, nullable)
- refreshToken (text, nullable)
- expiresAt (timestamp, nullable)

Use Drizzle ORM's pgTable function to define these schemas.

### Step 1.3: Database Connection

*File:* auth-backend/src/db/index.ts

*Requirements:*
1. Import drizzle from drizzle-orm/postgres-js
2. Import postgres driver
3. Read DATABASE_URL from environment variables
4. Create postgres connection
5. Create drizzle instance with the schema
6. Export the db instance

### Step 1.4: Better Auth Configuration

*File:* auth-backend/src/lib/auth.ts

*Configuration Requirements:*

1. *Import Better Auth:*
   - Import betterAuth from better-auth
   - Import drizzleAdapter from better-auth/adapters/drizzle

2. *Configure Better Auth with these settings:*

   *Database Adapter:*
   - Use drizzleAdapter with the db instance
   - Provider: "pg" (PostgreSQL)
   - Schema: map user, session, and account tables

   *Email & Password Authentication:*
   - Enable email/password authentication
   - Set requireEmailVerification: false (for development, change in production)

   *Session Configuration:*
   - Session expires in 7 days (60 * 60 * 24 * 7 seconds)
   - Update age: 1 day
   - Enable cookie cache with 5-minute max age

   *Additional User Fields:*
   - Register ALL custom background fields as additionalFields
   - Each field must have:
     - type: "string" or "boolean"
     - required: true/false
     - input: true (allows input during signup)

   *Trusted Origins:*
   - Add http://localhost:3000 (Docusaurus dev)
   - Add http://localhost:3001 (alternative port)
   - Add process.env.FRONTEND_URL from .env

3. *Export:*
   - Export the configured auth instance
   - Export type Session for TypeScript

### Step 1.5: Express Server Setup

*File:* auth-backend/src/index.ts

*Server Requirements:*

1. *Import Dependencies:*
   - Import express
   - Import cors
   - Import the auth instance from ./lib/auth

2. *Middleware Setup:*
   - Use express.json() for JSON parsing
   - Configure CORS with:
     - origin: Frontend URL from env
     - credentials: true (CRITICAL for cookies)

3. *Better Auth Routes:*
   - Mount Better Auth handler at /api/auth/*
   - Use auth.handler(req, res) for all auth routes
   - This automatically creates:
     - /api/auth/sign-up/email (signup endpoint)
     - /api/auth/sign-in/email (login endpoint)
     - /api/auth/sign-out (logout endpoint)
     - /api/auth/session (get session endpoint)

4. *Custom API Routes (Optional but Recommended):*
   - GET /api/user/profile - Get current user profile
     - Verify session using auth.api.getSession()
     - Return user data if authenticated
     - Return 401 if not authenticated

   - PATCH /api/user/background - Update user background
     - Verify session
     - Update user fields in database
     - Return updated user

5. *Server Startup:*
   - Listen on port from process.env.PORT or default 5000
   - Log server URL on startup

### Step 1.6: Environment Variables

*File:* auth-backend/.env

*Required Variables:*

DATABASE_URL=postgresql://[user]:[password]@[host]/[database]?sslmode=require
BETTER_AUTH_SECRET=[generate 32+ character random string]
BETTER_AUTH_URL=http://localhost:5000
FRONTEND_URL=http://localhost:3000
NODE_ENV=development
PORT=5000


*Important Notes:*
- BETTER_AUTH_SECRET must be at least 32 characters
- Never commit .env file to git
- Add .env to .gitignore

### Step 1.7: Database Migration

*File:* auth-backend/drizzle.config.ts

*Configuration:*
1. Export drizzle config with:
   - Schema path: ./src/db/schema.ts
   - Output directory: ./drizzle
   - Driver: pg
   - Database credentials from env

*Migration Commands:*
1. Generate migrations: npm run db:generate
2. Apply migrations: npm run db:migrate
3. Must run before starting server

### Step 1.8: Package Scripts

**Add to auth-backend/package.json:**
- "dev": Run development server with hot reload (using tsx)
- "build": Compile TypeScript to JavaScript
- "start": Run production server
- "db:generate": Generate database migrations
- "db:migrate": Apply migrations to database

---

## PHASE 2: FRONTEND SETUP

### Step 2.1: Install Dependencies

*Location:* C:\new\physical-ai-robotics-textbook\docusaurus

*Required Dependencies:*
- better-auth (same version as backend)
- @better-auth/react (React hooks for Better Auth)
- axios (for API calls, optional)

*Installation Command:*
Run in docusaurus folder: npm install better-auth @better-auth/react axios

### Step 2.2: Auth Client Setup

*File:* docusaurus/src/lib/auth-client.ts

*Purpose:* Create and configure the Better Auth client for frontend

*Requirements:*
1. Import createAuthClient from better-auth/react
2. Create auth client with:
   - baseURL: Backend URL (from env or hardcoded for dev)
   - Example: http://localhost:5000
3. Export the client and these hooks:
   - signIn - for login
   - signUp - for signup
   - signOut - for logout
   - useSession - React hook to get current session

*Environment Variable:*
- Add REACT_APP_AUTH_URL=http://localhost:5000 to Docusaurus env

---

## PHASE 3: AUTHENTICATION UI COMPONENTS

### Step 3.1: Signup Modal Component

*File:* docusaurus/src/components/auth/SignupModal.tsx

*Component Purpose:*
A two-step modal for user registration with background information collection.

*Props Interface:*
- isOpen (boolean) - controls modal visibility
- onClose (function) - closes the modal
- onSwitchToLogin (function) - switches to login modal

*State Requirements:*
1. step (number) - current step (1 or 2)
2. formData (object) - all form fields
3. error (string) - error messages
4. loading (boolean) - submission state

*Form Data Structure:*

- name (string)
- email (string)
- password (string)
- confirmPassword (string)
- softwareBackground (string)
- hardwareBackground (string)
- programmingLanguages (array of strings)
- roboticsExperience (string)
- aiMlExperience (string)
- hasRosExperience (boolean)
- hasGpuAccess (boolean)
- learningGoals (string)


*Step 1 - Account Information:*
Display fields for:
- Full Name (required)
- Email (required)
- Password (required, min 8 characters)
- Confirm Password (required, must match)

Validation:
- All fields filled
- Passwords match
- Password minimum 8 characters
- Valid email format

Button: "Next: Background Info" - validates and moves to step 2

*Step 2 - Background Information:*

Display fields for:

1. *Software Background* (dropdown, required):
   - Options: Beginner, Intermediate, Advanced, Expert
   - Label: "Software Background"

2. *Hardware Background* (dropdown, required):
   - Options: None, Basic, Intermediate, Advanced
   - Label: "Hardware Background"

3. *Programming Languages* (checkboxes, optional):
   - Options: Python, C++, JavaScript, Java, C, ROS2
   - Display in 2-column grid
   - Can select multiple

4. *Robotics Experience* (dropdown, required):
   - Options: None, Hobbyist, Academic, Professional
   - Label: "Robotics Experience"

5. *AI/ML Experience* (dropdown, required):
   - Options: None, Basic, Intermediate, Advanced
   - Label: "AI/ML Experience"

6. *ROS Experience* (checkbox):
   - Label: "I have ROS/ROS2 experience"
   - Boolean field

7. *GPU Access* (checkbox):
   - Label: "I have access to GPU (NVIDIA RTX 4070+)"
   - Boolean field

8. *Learning Goals* (textarea, optional):
   - Label: "Learning Goals"
   - Placeholder: "What do you want to achieve with this course?"
   - 3 rows

Buttons:
- "Back" - returns to step 1
- "Complete Signup" - submits form

*Validation:*
- All required fields must be filled
- Show error if validation fails

*Submission:*
1. Call signUp.email() from auth client
2. Pass all form data including background fields
3. For programmingLanguages, convert array to JSON string
4. On success: close modal and reload page
5. On error: display error message

*UI Features:*
- Step indicator showing "Step 1 of 2" or "Step 2 of 2"
- Loading state on submit button
- Error message display
- Close button (X) in top-right
- "Already have an account? Login" link at bottom

*Styling:*
- Modal overlay with backdrop blur
- Centered modal with max-width 500px
- Responsive design
- Professional form styling
- Proper spacing and typography

### Step 3.2: Login Modal Component

*File:* docusaurus/src/components/auth/LoginModal.tsx

*Component Purpose:*
Simple login modal for existing users.

*Props Interface:*
- isOpen (boolean)
- onClose (function)
- onSwitchToSignup (function)

*State Requirements:*
- email (string)
- password (string)
- error (string)
- loading (boolean)

*Form Fields:*
1. Email (required, type="email")
2. Password (required, type="password")

*Submission:*
1. Call signIn.email() with email and password
2. On success: close modal and reload page
3. On error: display "Login failed" message

*UI Features:*
- Title: "Welcome Back"
- Subtitle: "Login to continue your learning journey"
- Error message display
- Loading state on submit button
- "Don't have an account? Sign up" link at bottom
- Close button (X)

*Styling:*
- Same modal design as signup
- Professional and clean

### Step 3.3: Auth Buttons Component

*File:* docusaurus/src/components/auth/AuthButtons.tsx

*Component Purpose:*
Displays login/signup buttons OR user profile dropdown based on auth state.

*State Requirements:*
- showSignup (boolean) - signup modal visibility
- showLogin (boolean) - login modal visibility
- showDropdown (boolean) - user dropdown visibility

*Session Hook:*
- Use useSession() hook from auth client
- Get data (session info) and isPending (loading state)

*Conditional Rendering:*

**If Loading (isPending):**
- Show "Loading..." text

**If Not Authenticated (!session):**
Display two buttons:
1. *Login Button:*
   - Text: "Login"
   - Style: Outlined/ghost button
   - onClick: open login modal

2. *Signup Button:*
   - Text: "Sign Up"
   - Style: Primary/solid button
   - onClick: open signup modal

**If Authenticated (session.user):**
Display user menu:
1. *User Button:*
   - Show user's name
   - Show avatar (first letter of name in circle)
   - onClick: toggle dropdown

2. *Dropdown Menu (when open):*
   - Show user email
   - Divider line
   - "Logout" button that calls signOut() and reloads

*Modal Integration:*
- Include both SignupModal and LoginModal components
- Pass appropriate state and handlers
- Handle modal switching between signup/login

*Styling:*
- Match Docusaurus navbar style
- Responsive button sizing
- Smooth transitions
- Professional dropdown with shadow

### Step 3.4: CSS Modules

*Files:*
- docusaurus/src/components/auth/AuthModal.module.css
- docusaurus/src/components/auth/AuthButtons.module.css

*AuthModal.module.css Requirements:*

*Modal Layout:*
- Overlay: fixed position, full viewport, dark backdrop (rgba(0,0,0,0.7))
- Content: centered, max-width 500px, max-height 90vh, overflow-y auto
- White/theme background, rounded corners (12px)
- Box shadow for depth
- Padding: 32px
- Close button: absolute top-right, 32px font-size

*Form Styling:*
- Form groups with 20px bottom margin
- Labels: bold, 8px bottom margin
- Inputs: full width, 10px padding, 2px border, rounded (6px)
- Focus state: primary color border
- Select/textarea: same styling as inputs

*Buttons:*
- Primary: full width, primary color background, white text, rounded
- Secondary: gray background for "Back" button
- Hover states with smooth transitions
- Disabled state: reduced opacity, no pointer

*Step Indicator:*
- Light primary background
- 8px padding, rounded
- Small font (14px), bold

*Error Messages:*
- Red background (#fee)
- Red text (#c33)
- Padding, rounded corners

*Checkboxes:*
- Grid layout for programming languages (2 columns)
- Checkbox labels with flex layout

*Switch Auth Link:*
- Centered text
- Button styled as link (underlined, primary color)

*AuthButtons.module.css Requirements:*

*Button Layouts:*
- Flex container with gap
- Login: outlined style with primary border
- Signup: solid primary background
- Hover effects with color transitions

*User Menu:*
- User button: flex layout, border, rounded
- Avatar: 32px circle, primary background, centered letter
- Dropdown: absolute positioning, shadow, rounded
- Dropdown items: padding, hover effect

*Loading State:*
- Simple text with muted color

*Responsive:*
- Works on mobile and desktop
- Proper touch targets

---

## PHASE 4: NAVBAR INTEGRATION

### Step 4.1: Update Docusaurus Config

*File:* docusaurus/docusaurus.config.js

*Approach 1: Theme Swizzling (Recommended)*

*Do NOT manually edit the config file for navbar items.*

Instead, swizzle the Navbar component:

*Steps:*
1. Run swizzle command: npm run swizzle @docusaurus/theme-classic Navbar/Content -- --wrap
2. This creates: docusaurus/src/theme/Navbar/Content/index.tsx
3. In that file:
   - Import original Content component
   - Import your AuthButtons component
   - Render both in wrapper

*Wrapper Component Structure:*

export default function ContentWrapper(props) {
  return (
    <>
      <Content {...props} />
      <AuthButtons />
    </>
  );
}


This ensures AuthButtons appear in navbar without breaking Docusaurus structure.

*Approach 2: Custom Navbar Item (Alternative)*

If swizzling doesn't work, you can:
1. Keep existing navbar items in config
2. Add a custom component at the end
3. Manually render AuthButtons in a custom page component

### Step 4.2: Verify Navbar Integration

*Testing Checklist:*
- [ ] Auth buttons appear on the right side of navbar
- [ ] Buttons are properly styled and match navbar theme
- [ ] Login/Signup modals open when buttons clicked
- [ ] User dropdown appears when logged in
- [ ] Logout works correctly
- [ ] Works on mobile (responsive)

---

## PHASE 5: TESTING & VALIDATION

### Step 5.1: Backend Testing

*Start Backend Server:*
1. Navigate to auth-backend folder
2. Run npm run dev
3. Verify server starts on port 5000
4. Check console for any errors

*Test Endpoints Manually:*

Using tools like Postman or cURL:

1. *Test Signup:*
   - POST to http://localhost:5000/api/auth/sign-up/email
   - Body (JSON):
   json
   {
     "email": "test@example.com",
     "password": "Test1234",
     "name": "Test User",
     "softwareBackground": "intermediate",
     "hardwareBackground": "basic",
     "programmingLanguages": "[\"Python\", \"JavaScript\"]",
     "roboticsExperience": "hobbyist",
     "aiMlExperience": "basic",
     "hasRosExperience": false,
     "hasGpuAccess": true,
     "learningGoals": "Learn robotics"
   }

   - Expected: 200 OK with user object

2. *Test Login:*
   - POST to http://localhost:5000/api/auth/sign-in/email
   - Body:
   json
   {
     "email": "test@example.com",
     "password": "Test1234"
   }

   - Expected: 200 OK with session cookie

3. *Test Session:*
   - GET to http://localhost:5000/api/auth/session
   - Include session cookie from login
   - Expected: User session data

4. *Test Logout:*
   - POST to http://localhost:5000/api/auth/sign-out
   - Expected: Session cleared

*Database Verification:*
1. Connect to Neon database
2. Query users table
3. Verify user created with all background fields
4. Check sessions table for active sessions

### Step 5.2: Frontend Testing

*Start Docusaurus:*
1. Navigate to docusaurus folder
2. Run npm start
3. Verify site opens on port 3000

*UI Testing Checklist:*

*Modal Display:*
- [ ] Signup modal opens when "Sign Up" clicked
- [ ] Login modal opens when "Login" clicked
- [ ] Modals close when X clicked
- [ ] Modals close when clicking outside
- [ ] Can switch between signup and login modals

*Signup Flow:*
- [ ] Step 1 displays all account fields
- [ ] Email validation works
- [ ] Password validation works (8+ chars)
- [ ] Password confirmation matches
- [ ] "Next" button moves to step 2
- [ ] Step indicator shows "Step 1 of 2"

- [ ] Step 2 displays all background fields
- [ ] All dropdowns have correct options
- [ ] Programming language checkboxes work
- [ ] ROS/GPU checkboxes work
- [ ] "Back" button returns to step 1
- [ ] Step indicator shows "Step 2 of 2"

- [ ] Submit button shows loading state
- [ ] Successful signup closes modal and reloads
- [ ] User appears logged in after signup
- [ ] Error messages display correctly

*Login Flow:*
- [ ] Email and password fields work
- [ ] Correct credentials log user in
- [ ] Wrong credentials show error
- [ ] Login button shows loading state
- [ ] Successful login closes modal and reloads

*Authenticated State:*
- [ ] User name appears in navbar
- [ ] User avatar shows first letter of name
- [ ] Dropdown opens when clicked
- [ ] User email shown in dropdown
- [ ] Logout button works
- [ ] After logout, shows login/signup buttons again

*Error Handling:*
- [ ] Network errors display user-friendly messages
- [ ] Validation errors are clear
- [ ] Form doesn't submit with missing required fields

### Step 5.3: Integration Testing

*Cross-Component Tests:*

1. *Full Signup to Dashboard Flow:*
   - Sign up new user with all fields
   - Verify user logged in
   - Navigate to different pages
   - Verify user stays logged in
   - Refresh page
   - Verify session persists

2. *Session Persistence:*
   - Log in
   - Close browser tab
   - Reopen site
   - Verify still logged in (within 7 days)

3. *Multi-Tab Behavior:*
   - Log in on tab 1
   - Open site in tab 2
   - Verify logged in on tab 2
   - Log out on tab 1
   - Verify logged out on tab 2 (may need refresh)

### Step 5.4: Security Testing

*Checklist:*
- [ ] Passwords are hashed (not stored in plain text)
- [ ] Session cookies are HTTP-only
- [ ] CORS is properly configured
- [ ] CSRF protection enabled (Better Auth default)
- [ ] SQL injection prevented (Drizzle ORM handles this)
- [ ] XSS protection (React handles this)
- [ ] Environment variables not exposed to frontend
- [ ] .env file not committed to git

---

## PHASE 6: DEPLOYMENT PREPARATION

### Step 6.1: Environment Variables for Production

*Backend (.env.production):*

DATABASE_URL=[production Neon database URL]
BETTER_AUTH_SECRET=[new 32+ character secret for production]
BETTER_AUTH_URL=[your production backend URL]
FRONTEND_URL=[your production frontend URL]
NODE_ENV=production
PORT=5000


*Frontend Environment:*
- Set REACT_APP_AUTH_URL to production backend URL
- Update in Vercel/GitHub Pages deployment settings

### Step 6.2: Database Migration for Production

*Steps:*
1. Create production database on Neon
2. Update DATABASE_URL in production env
3. Run migrations: npm run db:migrate
4. Verify tables created

### Step 6.3: Security Hardening

*Before Production:*
1. Change requireEmailVerification to true in Better Auth config
2. Set secure cookie flags (HTTPS only)
3. Enable rate limiting on auth endpoints
4. Add CAPTCHA to signup if needed
5. Set up email service for verification

### Step 6.4: Backend Deployment

*Recommended Platforms:*
- Railway
- Render
- Heroku
- Vercel (serverless)

*Deployment Steps:*
1. Push code to GitHub
2. Connect repository to platform
3. Set all environment variables
4. Configure build command: npm run build
5. Configure start command: npm start
6. Deploy and test

### Step 6.5: Frontend Deployment

*GitHub Pages:*
1. Build: npm run build in docusaurus folder
2. Deploy build folder to gh-pages branch

*Vercel:*
1. Connect GitHub repo
2. Framework: Docusaurus
3. Build command: npm run build
4. Output directory: build
5. Set environment variables

### Step 6.6: Post-Deployment Testing

*Full Flow Test:*
1. Visit production site
2. Click "Sign Up"
3. Complete both signup steps with all fields
4. Verify account created
5. Log out
6. Log back in
7. Verify session persists
8. Test on mobile device
9. Test on different browsers

---

## TROUBLESHOOTING GUIDE

### Common Issues & Solutions

*1. "Better Auth not installed" Error:*
- Solution: Run npm install better-auth in both frontend and backend

*2. CORS Error in Browser Console:*
- Check backend CORS configuration
- Verify credentials: true is set
- Ensure frontend URL is in trustedOrigins
- Check browser Network tab for preflight OPTIONS request

*3. "Database connection failed":*
- Verify DATABASE_URL is correct
- Check Neon database is running
- Verify firewall/network access
- Test connection string with psql or database client

*4. "Session not persisting":*
- Check cookies in browser DevTools
- Verify cookie domain matches
- Ensure HTTPS in production
- Check session expiry settings

*5. "Additional fields not saving":*
- Verify fields are in database schema
- Check Better Auth config has additionalFields
- Ensure input: true is set for each field
- Check database column types match

*6. "Modal not opening":*
- Check React state management
- Verify button onClick handlers
- Check browser console for errors
- Ensure CSS z-index is high enough

*7. "TypeScript errors":*
- Run npm install @types/node @types/react
- Check tsconfig.json is properly configured
- Verify all imports have correct types

*8. "Build fails on deployment":*
- Check all dependencies are in package.json (not devDependencies)
- Verify environment variables are set
- Check build logs for specific errors
- Test build locally first: npm run build

---

## SUCCESS CRITERIA

Your implementation is complete when:

- [ ] Backend server runs without errors
- [ ] Database tables created successfully
- [ ] Signup modal collects ALL required background fields
- [ ] Signup creates user in database with background data
- [ ] Login works with correct credentials
- [ ] Session persists across page reloads
- [ ] Logout clears session properly
- [ ] Auth buttons appear in Docusaurus navbar
- [ ] User dropdown shows when logged in
- [ ] All modals have professional styling
- [ ] No console errors in browser
- [ ] Mobile responsive design works
- [ ] Can deploy to production successfully

---

## BONUS POINTS PREPARATION

This authentication system sets up the foundation for:

*Personalization (50 bonus points):*
- User background data is stored and accessible
- Can fetch user data using useSession() hook
- Ready to customize content based on:
  - softwareBackground
  - hardwareBackground
  - roboticsExperience
  - aiMlExperience
  - hasRosExperience
  - hasGpuAccess

*Translation (50 bonus points):*
- Logged-in user can be identified
- User preferences can be stored
- Ready to add translation toggle per user

---

## FINAL NOTES

*Code Quality:*
- Write clean, readable code
- Add comments for complex logic
- Use TypeScript for type safety
- Follow React best practices
- Use proper error handling

*User Experience:*
- Fast load times
- Smooth animations
- Clear error messages
- Intuitive navigation
- Responsive design

*Security:*
- Never log sensitive data
- Sanitize all user inputs
- Use environment variables
- Keep dependencies updated
- Follow OWASP guidelines

*Documentation:*
- Add README.md to both folders
- Document API endpoints
- Explain environment variables
- Include setup instructions

---