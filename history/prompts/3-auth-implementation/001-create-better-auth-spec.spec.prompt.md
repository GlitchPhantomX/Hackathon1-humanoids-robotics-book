---
id: 001
title: Create Better Auth Spec
stage: spec
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: auth-implementation
branch: 3-auth-implementation
user: Claude
command: /sp.specify
labels: [auth, specification, docusaurus, better-auth]
links:
  spec: C:/new/specs/3-auth-implementation/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - C:/new/specs/3-auth-implementation/spec.md
 - C:/new/specs/3-auth-implementation/checklists/requirements.md
tests:
 - null
---

## Prompt

create my specification in specs folder named /specs/3-auth-specification # Better Auth Implementation Specification
**Version:** 1.0.0
**Project:** Physical AI & Robotics Textbook Authentication System
**Target Framework:** Docusaurus + Better Auth
**Database:** Neon PostgreSQL

---

## 1. Executive Summary

### 1.1 Project Overview
Implement a production-ready authentication system for a Docusaurus-based educational platform using Better Auth library. The system will collect comprehensive user background information during registration to enable future content personalization.

### 1.2 Technology Stack
- **Frontend:** React (Docusaurus), Better Auth React client
- **Backend:** Node.js, Express, Better Auth
- **Database:** PostgreSQL (Neon), Drizzle ORM
- **Authentication:** Better Auth with email/password strategy

### 1.3 Success Metrics
- User registration with 12 background data points
- Session persistence for 7 days
- Sub-200ms authentication response time
- Zero authentication-related security vulnerabilities

---

## 2. System Architecture

### 2.1 Project Structure
```
physical-ai-robotics-textbook/
├── docusaurus/                      # Frontend application
│   ├── src/
│   │   ├── components/auth/         # Authentication UI components
│   │   ├── lib/auth-client.ts       # Better Auth client configuration
│   │   └── theme/Navbar/            # Swizzled navbar (optional)
│   ├── docusaurus.config.js
│   └── package.json
│
└── auth-backend/                    # Authentication server
    ├── src/
    │   ├── index.ts                 # Express server entry point
    │   ├── lib/auth.ts              # Better Auth configuration
    │   └── db/
    │       ├── schema.ts            # Drizzle schema definitions
    │       └── index.ts             # Database connection
    ├── drizzle.config.ts
    ├── .env
    └── package.json
```

### 2.2 Data Flow
```
User → Docusaurus UI → Auth Client → Express Server → Better Auth → Database
                                                                       ↓
                                              Session Cookie ← ← ← ← ← ←
```

---

## 3. Backend Specification

### 3.1 Environment Configuration

**File:** `auth-backend/.env`

```env
# Database
DATABASE_URL=postgresql://user:password@host/database?sslmode=require

# Better Auth
BETTER_AUTH_SECRET=<32+ character random string>
BETTER_AUTH_URL=http://localhost:5000
FRONTEND_URL=http://localhost:3000

# Server
NODE_ENV=development
PORT=5000
```

**Security Requirements:**
- `BETTER_AUTH_SECRET` MUST be cryptographically random, minimum 32 characters
- Never commit `.env` to version control
- Use separate secrets for development/staging/production

### 3.2 Database Schema

**File:** `auth-backend/src/db/schema.ts`

#### 3.2.1 Users Table
```typescript
import { pgTable, text, boolean, timestamp, jsonb } from 'drizzle-orm/pg-core';

export const users = pgTable('users', {
  // Better Auth required fields
  id: text('id').primaryKey(),
  email: text('email').notNull().unique(),
  emailVerified: boolean('email_verified').notNull().default(false),
  name: text('name').notNull(),
  image: text('image'),
  createdAt: timestamp('created_at').notNull().defaultNow(),
  updatedAt: timestamp('updated_at').notNull().defaultNow(),

  // Custom background fields for personalization
  softwareBackground: text('software_background', {
    enum: ['beginner', 'intermediate', 'advanced', 'expert']
  }).notNull(),

  hardwareBackground: text('hardware_background', {
    enum: ['none', 'basic', 'intermediate', 'advanced']
  }).notNull(),

  programmingLanguages: jsonb('programming_languages').$type<string[]>(),

  roboticsExperience: text('robotics_experience', {
    enum: ['none', 'hobbyist', 'academic', 'professional']
  }).notNull(),

  aiMlExperience: text('ai_ml_experience', {
    enum: ['none', 'basic', 'intermediate', 'advanced']
  }).notNull(),

  hasRosExperience: boolean('has_ros_experience').notNull().default(false),
  hasGpuAccess: boolean('has_gpu_access').notNull().default(false),
  learningGoals: text('learning_goals'),
});
```

#### 3.2.2 Sessions Table
```typescript
export const sessions = pgTable('sessions', {
  id: text('id').primaryKey(),
  userId: text('user_id').notNull().references(() => users.id, { onDelete: 'cascade' }),
  expiresAt: timestamp('expires_at').notNull(),
  token: text('token').notNull().unique(),
  ipAddress: text('ip_address'),
  userAgent: text('user_agent'),
});
```

#### 3.2.3 Accounts Table
```typescript
export const accounts = pgTable('accounts', {
  id: text('id').primaryKey(),
  userId: text('user_id').notNull().references(() => users.id, { onDelete: 'cascade' }),
  accountId: text('account_id').notNull(),
  providerId: text('provider_id').notNull(),
  accessToken: text('access_token'),
  refreshToken: text('refresh_token'),
  expiresAt: timestamp('expires_at'),
});
```

### 3.3 Database Connection

**File:** `auth-backend/src/db/index.ts`

```typescript
import { drizzle } from 'drizzle-orm/postgres-js';
import postgres from 'postgres';
import * as schema from './schema';

const connectionString = process.env.DATABASE_URL!;

const client = postgres(connectionString);
export const db = drizzle(client, { schema });
```

### 3.4 Better Auth Configuration

**File:** `auth-backend/src/lib/auth.ts`

```typescript
import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from '../db';
import * as schema from '../db/schema';

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
    schema: {
      user: schema.users,
      session: schema.sessions,
      account: schema.accounts,
    }
  }),

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minutes
    }
  },

  user: {
    additionalFields: {
      softwareBackground: {
        type: 'string',
        required: true,
        input: true,
      },
      hardwareBackground: {
        type: 'string',
        required: true,
        input: true,
      },
      programmingLanguages: {
        type: 'string', // JSON string
        required: false,
        input: true,
      },
      roboticsExperience: {
        type: 'string',
        required: true,
        input: true,
      },
      aiMlExperience: {
        type: 'string',
        required: true,
        input: true,
      },
      hasRosExperience: {
        type: 'boolean',
        required: true,
        input: true,
      },
      hasGpuAccess: {
        type: 'boolean',
        required: true,
        input: true,
      },
      learningGoals: {
        type: 'string',
        required: false,
        input: true,
      },
    }
  },

  trustedOrigins: [
    'http://localhost:3000',
    'http://localhost:3001',
    process.env.FRONTEND_URL || '',
  ],
});

export type Session = typeof auth.$Infer.Session;
```

### 3.5 Express Server

**File:** `auth-backend/src/index.ts`

```typescript
import express from 'express';
import cors from 'cors';
import { auth } from './lib/auth';

const app = express();

// Middleware
app.use(express.json());
app.use(cors({
  origin: process.env.FRONTEND_URL || 'http://localhost:3000',
  credentials: true, // CRITICAL: Required for cookies
}));

// Better Auth routes
app.all('/api/auth/*', (req, res) => auth.handler(req, res));

// Optional: Custom profile endpoint
app.get('/api/user/profile', async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers });
    if (!session) {
      return res.status(401).json({ error: 'Unauthorized' });
    }
    res.json({ user: session.user });
  } catch (error) {
    res.status(500).json({ error: 'Internal server error' });
  }
});

// Server startup
const PORT = process.env.PORT || 5000;
app.listen(PORT, () => {
  console.log(`Auth server running on http://localhost:${PORT}`);
});
```

### 3.6 Package Configuration

**File:** `auth-backend/package.json`

```json
{
  "name": "auth-backend",
  "version": "1.0.0",
  "type": "module",
  "scripts": {
    "dev": "tsx watch src/index.ts",
    "build": "tsc",
    "start": "node dist/index.js",
    "db:generate": "drizzle-kit generate",
    "db:migrate": "drizzle-kit migrate"
  },
  "dependencies": {
    "better-auth": "^1.0.0",
    "express": "^4.18.2",
    "cors": "^2.8.5",
    "drizzle-orm": "^0.33.0",
    "postgres": "^3.4.3",
    "dotenv": "^16.4.5"
  },
  "devDependencies": {
    "@types/express": "^4.17.21",
    "@types/cors": "^2.8.17",
    "@types/node": "^20.11.0",
    "tsx": "^4.7.0",
    "typescript": "^5.3.3",
    "drizzle-kit": "^0.24.0"
  }
}
```

### 3.7 Drizzle Configuration

**File:** `auth-backend/drizzle.config.ts`

```typescript
import { defineConfig } from 'drizzle-kit';
import dotenv from 'dotenv';

dotenv.config();

export default defineConfig({
  schema: './src/db/schema.ts',
  out: './drizzle',
  dialect: 'postgresql',
  dbCredentials: {
    url: process.env.DATABASE_URL!,
  },
});
```

---

## 4. Frontend Specification

### 4.1 Auth Client Setup

**File:** `docusaurus/src/lib/auth-client.ts`

```typescript
import { createAuthClient } from '@better-auth/react';

const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_URL || 'http://localhost:5000',
});

export const { signIn, signUp, signOut, useSession } = authClient;
export { authClient };
```

**Environment Variable:**
Add to `.env` or `.env.local`:
```
REACT_APP_AUTH_URL=http://localhost:5000
```

### 4.2 Component Specifications

#### 4.2.1 Signup Modal Component

**File:** `docusaurus/src/components/auth/SignupModal.tsx`

**Props Interface:**
```typescript
interface SignupModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToLogin: () => void;
}
```

**State Interface:**
```typescript
interface FormData {
  // Step 1: Account Information
  name: string;
  email: string;
  password: string;
  confirmPassword: string;

  // Step 2: Background Information
  softwareBackground: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  hardwareBackground: 'none' | 'basic' | 'intermediate' | 'advanced';
  programmingLanguages: string[];
  roboticsExperience: 'none' | 'hobbyist' | 'academic' | 'professional';
  aiMlExperience: 'none' | 'basic' | 'intermediate' | 'advanced';
  hasRosExperience: boolean;
  hasGpuAccess: boolean;
  learningGoals: string;
}
```

**Validation Rules:**

*Step 1:*
- Name: Required, minimum 2 characters
- Email: Required, valid email format
- Password: Required, minimum 8 characters
- Confirm Password: Must match password

*Step 2:*
- Software Background: Required
- Hardware Background: Required
- Programming Languages: Optional (array)
- Robotics Experience: Required
- AI/ML Experience: Required
- ROS Experience: Boolean (default false)
- GPU Access: Boolean (default false)
- Learning Goals: Optional, max 500 characters

**Form Fields Configuration:**

*Step 1 Fields:*
```typescript
const step1Fields = [
  { name: 'name', label: 'Full Name', type: 'text', required: true },
  { name: 'email', label: 'Email', type: 'email', required: true },
  { name: 'password', label: 'Password', type: 'password', required: true },
  { name: 'confirmPassword', label: 'Confirm Password', type: 'password', required: true },
];
```

*Step 2 Fields:*
```typescript
const step2Fields = {
  softwareBackground: {
    label: 'Software Background',
    options: [
      { value: 'beginner', label: 'Beginner - New to programming' },
      { value: 'intermediate', label: 'Intermediate - 1-3 years experience' },
      { value: 'advanced', label: 'Advanced - 3-5 years experience' },
      { value: 'expert', label: 'Expert - 5+ years experience' },
    ],
  },
  hardwareBackground: {
    label: 'Hardware Background',
    options: [
      { value: 'none', label: 'None - No hardware experience' },
      { value: 'basic', label: 'Basic - Some tinkering experience' },
      { value: 'intermediate', label: 'Intermediate - Built projects' },
      { value: 'advanced', label: 'Advanced - Professional experience' },
    ],
  },
  programmingLanguages: {
    label: 'Programming Languages (Select all that apply)',
    options: ['Python', 'C++', 'JavaScript', 'Java', 'C', 'ROS2'],
  },
  roboticsExperience: {
    label: 'Robotics Experience',
    options: [
      { value: 'none', label: 'None - Complete beginner' },
      { value: 'hobbyist', label: 'Hobbyist - Personal projects' },
      { value: 'academic', label: 'Academic - Research or coursework' },
      { value: 'professional', label: 'Professional - Industry experience' },
    ],
  },
  aiMlExperience: {
    label: 'AI/ML Experience',
    options: [
      { value: 'none', label: 'None - No experience' },
      { value: 'basic', label: 'Basic - Used ML models' },
      { value: 'intermediate', label: 'Intermediate - Trained models' },
      { value: 'advanced', label: 'Advanced - ML engineering' },
    ],
  },
  hasRosExperience: {
    label: 'I have ROS/ROS2 experience',
    type: 'checkbox',
  },
  hasGpuAccess: {
    label: 'I have access to GPU (NVIDIA RTX 4070 or better)',
    type: 'checkbox',
  },
  learningGoals: {
    label: 'Learning Goals (Optional)',
    placeholder: 'What do you want to achieve with this course?',
    type: 'textarea',
    rows: 3,
  },
};
```

**Submission Logic:**
```typescript
const handleSubmit = async () => {
  try {
    setLoading(true);
    setError('');

    const response = await signUp.email({
      email: formData.email,
      password: formData.password,
      name: formData.name,
      softwareBackground: formData.softwareBackground,
      hardwareBackground: formData.hardwareBackground,
      programmingLanguages: JSON.stringify(formData.programmingLanguages),
      roboticsExperience: formData.roboticsExperience,
      aiMlExperience: formData.aiMlExperience,
      hasRosExperience: formData.hasRosExperience,
      hasGpuAccess: formData.hasGpuAccess,
      learningGoals: formData.learningGoals,
    });

    if (response.error) {
      setError(response.error.message);
    } else {
      onClose();
      window.location.reload(); // Refresh to show authenticated state
    }
  } catch (err) {
    setError('An unexpected error occurred');
  } finally {
    setLoading(false);
  }
};
```

#### 4.2.2 Login Modal Component

**File:** `docusaurus/src/components/auth/LoginModal.tsx`

**Props Interface:**
```typescript
interface LoginModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToSignup: () => void;
}
```

**State Interface:**
```typescript
interface LoginState {
  email: string;
  password: string;
  error: string;
  loading: boolean;
}
```

**Form Fields:**
```typescript
const fields = [
  { name: 'email', label: 'Email', type: 'email', required: true },
  { name: 'password', label: 'Password', type: 'password', required: true },
];
```

**Submission Logic:**
```typescript
const handleSubmit = async (e: React.FormEvent) => {
  e.preventDefault();
  setLoading(true);
  setError('');

  try {
    const response = await signIn.email({
      email,
      password,
    });

    if (response.error) {
      setError('Invalid email or password');
    } else {
      onClose();
      window.location.reload();
    }
  } catch (err) {
    setError('Login failed. Please try again.');
  } finally {
    setLoading(false);
  }
};
```

#### 4.2.3 Auth Buttons Component

**File:** `docusaurus/src/components/auth/AuthButtons.tsx`

**Component Logic:**
```typescript
export default function AuthButtons() {
  const { data: session, isPending } = useSession();
  const [showSignup, setShowSignup] = useState(false);
  const [showLogin, setShowLogin] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);

  if (isPending) {
    return <div className={styles.loading}>Loading...</div>;
  }

  if (!session?.user) {
    return (
      <>
        <div className={styles.authButtons}>
          <button
            className={styles.loginButton}
            onClick={() => setShowLogin(true)}
          >
            Login
          </button>
          <button
            className={styles.signupButton}
            onClick={() => setShowSignup(true)}
          >
            Sign Up
          </button>
        </div>

        <LoginModal
          isOpen={showLogin}
          onClose={() => setShowLogin(false)}
          onSwitchToSignup={() => {
            setShowLogin(false);
            setShowSignup(true);
          }}
        />

        <SignupModal
          isOpen={showSignup}
          onClose={() => setShowSignup(false)}
          onSwitchToLogin={() => {
            setShowSignup(false);
            setShowLogin(true);
          }}
        />
      </>
    );
  }

  // Authenticated user dropdown
  return (
    <div className={styles.userMenu}>
      <button
        className={styles.userButton}
        onClick={() => setShowDropdown(!showDropdown)}
      >
        <div className={styles.avatar}>
          {session.user.name[0].toUpperCase()}
        </div>
        <span>{session.user.name}</span>
      </button>

      {showDropdown && (
        <div className={styles.dropdown}>
          <div className={styles.userEmail}>{session.user.email}</div>
          <div className={styles.divider} />
          <button
            className={styles.logoutButton}
            onClick={async () => {
              await signOut();
              window.location.reload();
            }}
          >
            Logout
          </button>
        </div>
      )}
    </div>
  );
}
```

### 4.3 Styling Specifications

#### 4.3.1 Modal Styles

**File:** `docusaurus/src/components/auth/AuthModal.module.css`

```css
/* Modal Overlay */
.modalOverlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.7);
  backdrop-filter: blur(4px);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 9999;
  animation: fadeIn 0.2s ease-out;
}

@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}

/* Modal Content */
.modalContent {
  background: var(--ifm-background-color);
  border-radius: 12px;
  box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04);
  max-width: 500px;
  width: 90%;
  max-height: 90vh;
  overflow-y: auto;
  padding: 32px;
  position: relative;
  animation: slideIn 0.3s ease-out;
}

@keyframes slideIn {
  from {
    opacity: 0;
    transform: translateY(-20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* Close Button */
.closeButton {
  position: absolute;
  top: 16px;
  right: 16px;
  background: none;
  border: none;
  font-size: 24px;
  cursor: pointer;
  color: var(--ifm-color-emphasis-600);
  padding: 4px;
  line-height: 1;
  transition: color 0.2s;
}

.closeButton:hover {
  color: var(--ifm-color-emphasis-800);
}

/* Step Indicator */
.stepIndicator {
  background: var(--ifm-color-primary-lightest);
  color: var(--ifm-color-primary-dark);
  padding: 8px 16px;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 600;
  text-align: center;
  margin-bottom: 24px;
}

/* Form Elements */
.formGroup {
  margin-bottom: 20px;
}

.label {
  display: block;
  font-weight: 600;
  margin-bottom: 8px;
  color: var(--ifm-font-color-base);
  font-size: 14px;
}

.input,
.select,
.textarea {
  width: 100%;
  padding: 10px 12px;
  border: 2px solid var(--ifm-color-emphasis-300);
  border-radius: 6px;
  font-size: 14px;
  font-family: inherit;
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
  transition: border-color 0.2s;
}

.input:focus,
.select:focus,
.textarea:focus {
  outline: none;
  border-color: var(--ifm-color-primary);
}

.textarea {
  resize: vertical;
  min-height: 80px;
}

/* Checkbox Grid */
.checkboxGrid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 12px;
}

.checkboxLabel {
  display: flex;
  align-items: center;
  gap: 8px;
  cursor: pointer;
  padding: 8px;
  border-radius: 6px;
  transition: background 0.2s;
}

.checkboxLabel:hover {
  background: var(--ifm-color-emphasis-100);
}

.checkbox {
  width: 18px;
  height: 18px;
  cursor: pointer;
}

/* Single Checkbox */
.singleCheckbox {
  display: flex;
  align-items: flex-start;
  gap: 8px;
  padding: 12px;
  background: var(--ifm-color-emphasis-100);
  border-radius: 6px;
}

/* Buttons */
.buttonGroup {
  display: flex;
  gap: 12px;
  margin-top: 24px;
}

.button {
  flex: 1;
  padding: 12px 24px;
  border: none;
  border-radius: 6px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
  font-family: inherit;
}

.primaryButton {
  background: var(--ifm-color-primary);
  color: white;
}

.primaryButton:hover:not(:disabled) {
  background: var(--ifm-color-primary-dark);
  transform: translateY(-1px);
}

.secondaryButton {
  background: var(--ifm-color-emphasis-300);
  color: var(--ifm-font-color-base);
}

.secondaryButton:hover:not(:disabled) {
  background: var(--ifm-color-emphasis-400);
}

.button:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

/* Error Message */
.error {
  background: #fee;
  color: #c33;
  padding: 12px;
  border-radius: 6px;
  margin-bottom: 16px;
  font-size: 14px;
}

/* Switch Auth Link */
.switchAuth {
  text-align: center;
  margin-top: 24px;
  font-size: 14px;
}

.switchAuthButton {
  background: none;
  border: none;
  color: var(--ifm-color-primary);
  text-decoration: underline;
  cursor: pointer;
  font-family: inherit;
  font-size: inherit;
}

.switchAuthButton:hover {
  color: var(--ifm-color-primary-dark);
}

/* Loading State */
.loading {
  text-align: center;
  padding: 40px;
  color: var(--ifm-color-emphasis-600);
}
```

#### 4.3.2 Auth Buttons Styles

**File:** `docusaurus/src/components/auth/AuthButtons.module.css`

```css
/* Auth Buttons Container */
.authButtons {
  display: flex;
  align-items: center;
  gap: 12px;
}

.loginButton,
.signupButton {
  padding: 8px 20px;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
  font-family: inherit;
  white-space: nowrap;
}

.loginButton {
  background: transparent;
  border: 2px solid var(--ifm-color-primary);
  color: var(--ifm-color-primary);
}

.loginButton:hover {
  background: var(--ifm-color-primary-lightest);
}

.signupButton {
  background: var(--ifm-color-primary);
  border: 2px solid var(--ifm-color-primary);
  color: white;
}

.signupButton:hover {
  background: var(--ifm-color-primary-dark);
  border-color: var(--ifm-color-primary-dark);
  transform: translateY(-1px);
}

/* User Menu */
.userMenu {
  position: relative;
}

.userButton {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 6px 12px;
  border: 2px solid var(--ifm-color-emphasis-300);
  border-radius: 6px;
  background: var(--ifm-background-color);
  cursor: pointer;
  transition: all 0.2s;
  font-family: inherit;
}

.userButton:hover {
  border-color: var(--ifm-color-primary);
}

.avatar {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  background: var(--ifm-color-primary);
  color: white;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  font-size: 14px;
}

/* Dropdown */
.dropdown {
  position: absolute;
  top: calc(100% + 8px);
  right: 0;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -2px rgba(0, 0, 0, 0.05);
  min-width: 200px;
  padding: 8px;
  z-index: 1000;
  animation: dropdownSlide 0.2s ease-out;
}

@keyframes dropdownSlide {
  from {
    opacity: 0;
    transform: translateY(-10px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.userEmail {
  padding: 8px 12px;
  font-size: 14px;
  color: var(--ifm-color-emphasis-600);
  word-break: break-word;
}

.divider {
  height: 1px;
  background: var(--ifm-color-emphasis-300);
  margin: 8px 0;
}

.logoutButton {
  width: 100%;
  padding: 8px 12px;
  border: none;
  background: transparent;
  color: #c33;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  border-radius: 4px;
  transition: background 0.2s;
  text-align: left;
  font-family: inherit;
}

.logoutButton:hover {
  background: #fee;
}

/* Loading State */
.loading {
  color: var(--ifm-color-emphasis-600);
  font-size: 14px;
  padding: 8px 12px;
}

/* Responsive */
@media (max-width: 768px) {
  .authButtons {
    gap: 8px;
  }

  .loginButton,
  .signupButton {
    padding: 6px 16px;
    font-size: 13px;
  }
}
```

### 4.4 Navbar Integration

#### Option 1: Theme Swizzling (Recommended)

**Command:**
```bash
cd docusaurus
npm run swizzle @docusaurus/theme-classic Navbar/Content -- --wrap
```

**File:** `docusaurus/src/theme/Navbar/Content/index.tsx`

```typescript
import React from 'react';
import Content from '@theme-original/Navbar/Content';
import AuthButtons from '@site/src/components/auth/AuthButtons';

export default function ContentWrapper(props) {
  return (
    <>
      <Content {...props} />
      <AuthButtons />
    </>
  );
}
```

#### Option 2: Manual Navbar Configuration

**File:** `docusaurus/docusaurus.config.js`

Add custom HTML item to navbar:

```javascript
module.exports = {
  // ... other config
  themeConfig: {
    navbar: {
      // ... existing navbar items
      items: [
        // ... your existing items
        {
          type: 'html',
          position: 'right',
          value: '<div id="auth-buttons-root"></div>',
        },
      ],
    },
  },
};
```

Then render AuthButtons into that div using a layout wrapper.

---

## 5. Implementation Workflow

### 5.1 Phase 1: Backend Setup (Estimated: 2-3 hours)

**Step 1: Initialize Backend Project**
```bash
cd C:\new\physical-ai-robotics-textbook
mkdir auth-backend
cd auth-backend
npm init -y
```

**Step 2: Install Dependencies**
```bash
npm install better-auth express cors drizzle-orm postgres dotenv
npm install -D @types/express @types/cors @types/node tsx typescript drizzle-kit
```

**Step 3: Configure TypeScript

Create `tsconfig.json`:
```json
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "moduleResolution": "node",
    "esModuleInterop": true,
    "strict": true,
    "skipLibCheck": true,
    "resolveJsonModule": true,
    "outDir": "./dist"
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules"]
}
```

**Step 4: Create Database Schema**
- Create `src/db/schema.ts` with all table definitions
- Ensure all Better Auth required fields are included
- Add all 8 custom background fields

**Step 5: Setup Database Connection**
- Create `src/db/index.ts` with Drizzle connection

**Step 6: Configure Better Auth**
- Create `src/lib/auth.ts`
- Configure database adapter
- Add all `additionalFields` for custom user data
- Set session expiry to 7 days

**Step 7: Build Express Server**
- Create `src/index.ts`
- Mount Better Auth handler at `/api/auth/*`
- Add optional custom profile endpoint

**Step 8: Setup Environment Variables**
- Create `.env` file (DO NOT COMMIT)
- Generate random 32+ character secret for `BETTER_AUTH_SECRET`
- Add Neon database connection string

**Step 9: Run Database Migrations**
```bash
npm run db:generate
npm run db:migrate
```

**Step 10: Test Backend**
```bash
npm run dev
```

Server should start on http://localhost:5000

### 5.2 Phase 2: Frontend Setup (Estimated: 2-3 hours)

**Step 1: Install Frontend Dependencies**
```bash
cd ../docusaurus
npm install better-auth @better-auth/react
```

**Step 2: Create Auth Client**
- Create `src/lib/auth-client.ts`
- Configure with backend URL
- Export hooks: `signIn`, `signUp`, `signOut`, `useSession`

**Step 3: Build Signup Modal**
- Create `src/components/auth/SignupModal.tsx`
- Implement 2-step form (Account → Background)
- Add validation for all fields
- Handle form submission with Better Auth

**Step 4: Build Login Modal**
- Create `src/components/auth/LoginModal.tsx`
- Simple email/password form
- Handle login with Better Auth

**Step 5: Build Auth Buttons**
- Create `src/components/auth/AuthButtons.tsx`
- Show login/signup buttons when not authenticated
- Show user dropdown when authenticated
- Handle logout functionality

**Step 6: Create Styles**
- Create `src/components/auth/AuthModal.module.css`
- Create `src/components/auth/AuthButtons.module.css`
- Ensure responsive design
- Match Docusaurus theme

**Step 7: Integrate with Navbar**
- Option A: Swizzle Navbar/Content component (recommended)
- Option B: Add custom HTML to navbar items
- Verify AuthButtons appear in navbar

**Step 8: Test Frontend**
```bash
npm start
```

Site should start on http://localhost:3000

### 5.3 Phase 3: Integration Testing (Estimated: 1-2 hours)

**Test Checklist:**

1. **Signup Flow:**
   - [ ] Open signup modal
   - [ ] Fill Step 1 with valid data
   - [ ] Validate passwords match
   - [ ] Progress to Step 2
   - [ ] Fill all background fields
   - [ ] Submit form successfully
   - [ ] User appears logged in
   - [ ] Check database for new user record

2. **Login Flow:**
   - [ ] Logout if logged in
   - [ ] Open login modal
   - [ ] Enter correct credentials
   - [ ] Submit successfully
   - [ ] User appears logged in

3. **Session Persistence:**
   - [ ] Login successfully
   - [ ] Refresh page
   - [ ] User still logged in
   - [ ] Navigate to different pages
   - [ ] Session persists

4. **Logout Flow:**
   - [ ] Click user dropdown
   - [ ] Click logout
   - [ ] User logged out
   - [ ] Auth buttons reappear

5. **Error Handling:**
   - [ ] Try signup with existing email → error shown
   - [ ] Try login with wrong password → error shown
   - [ ] Try signup with mismatched passwords → error shown
   - [ ] Network error → user-friendly message

6. **UI/UX:**
   - [ ] Modals are responsive on mobile
   - [ ] Forms are accessible (tab navigation)
   - [ ] Loading states work correctly
   - [ ] Animations are smooth
   - [ ] No console errors

---

## 6. Testing Procedures

### 6.1 Backend API Testing

Use Postman, Thunder Client, or cURL:

**Test 1: Signup**
```bash
POST http://localhost:5000/api/auth/sign-up/email
Content-Type: application/json

{
  "email": "test@example.com",
  "password": "SecurePass123",
  "name": "Test User",
  "softwareBackground": "intermediate",
  "hardwareBackground": "basic",
  "programmingLanguages": "[\"Python\", \"JavaScript\"]",
  "roboticsExperience": "hobbyist",
  "aiMlExperience": "basic",
  "hasRosExperience": false,
  "hasGpuAccess": true,
  "learningGoals": "Learn to build humanoid robots"
}
```

**Expected Response:** 200 OK with user object

**Test 2: Login**
```bash
POST http://localhost:5000/api/auth/sign-in/email
Content-Type: application/json

{
  "email": "test@example.com",
  "password": "SecurePass123"
}
```

**Expected Response:** 200 OK with session cookie

**Test 3: Get Session**
```bash
GET http://localhost:5000/api/auth/session
Cookie: <session_cookie_from_login>
```

**Expected Response:** User session data

**Test 4: Logout**
```bash
POST http://localhost:5000/api/auth/sign-out
Cookie: <session_cookie>
```

**Expected Response:** 200 OK, session cleared

### 6.2 Database Verification

Connect to Neon database and verify:

```sql
-- Check user was created with all fields
SELECT * FROM users WHERE email = 'test@example.com';

-- Verify session exists
SELECT * FROM sessions WHERE user_id = '<user_id>';

-- Check background data stored correctly
SELECT
  software_background,
  hardware_background,
  programming_languages,
  robotics_experience,
  ai_ml_experience,
  has_ros_experience,
  has_gpu_access,
  learning_goals
FROM users
WHERE email = 'test@example.com';
```

### 6.3 Security Testing

**Checklist:**
- [ ] Passwords are hashed (never stored plain text)
- [ ] Session cookies are HTTP-only
- [ ] Session cookies have Secure flag in production
- [ ] CORS properly configured (credentials: true)
- [ ] SQL injection impossible (ORM handles escaping)
- [ ] XSS protection (React escapes by default)
- [ ] Rate limiting on auth endpoints (production)
- [ ] CSRF protection enabled (Better Auth default)

---

## 7. Deployment Guide

### 7.1 Backend Deployment (Railway)

**Prerequisites:**
- GitHub repository
- Railway account

**Steps:**

1. **Prepare for Deployment:**
   ```bash
   cd auth-backend
   # Ensure build script in package.json
   # Ensure start script uses dist/index.js
   ```

2. **Push to GitHub:**
   ```bash
   git add .
   git commit -m "Add auth backend"
   git push origin main
   ```

3. **Deploy on Railway:**
   - Connect GitHub repository
   - Select `auth-backend` folder as root
   - Set environment variables in Railway dashboard:
     ```
     DATABASE_URL=<production_neon_url>
     BETTER_AUTH_SECRET=<new_production_secret>
     BETTER_AUTH_URL=https://<your-app>.railway.app
     FRONTEND_URL=https://<your-docusaurus-site>
     NODE_ENV=production
     ```
   - Deploy

4. **Run Migrations:**
   ```bash
   # In Railway console or locally with production DB
   npm run db:migrate
   ```

### 7.2 Frontend Deployment (Vercel/GitHub Pages)

**Update Environment Variable:**
```bash
# In Vercel dashboard or .env.production
REACT_APP_AUTH_URL=https://<your-backend>.railway.app
```

**Vercel Deployment:**
1. Connect GitHub repository
2. Framework preset: Docusaurus
3. Root directory: `docusaurus`
4. Build command: `npm run build`
5. Output directory: `build`
6. Add environment variable: `REACT_APP_AUTH_URL`
7. Deploy

**GitHub Pages Deployment:**
```bash
cd docusaurus
npm run build
# Deploy build/ directory to gh-pages branch
```

### 7.3 Production Checklist

- [ ] `BETTER_AUTH_SECRET` is different from development
- [ ] `requireEmailVerification` set to `true`
- [ ] SSL/HTTPS enabled on both frontend and backend
- [ ] CORS origins updated for production URLs
- [ ] Database backups configured
- [ ] Error logging/monitoring setup (e.g., Sentry)
- [ ] Rate limiting enabled
- [ ] Security headers configured
- [ ] Test complete signup/login flow in production

---

## 8. Troubleshooting Guide

### 8.1 Common Errors

**Error:** "CORS policy blocked"
**Cause:** Backend CORS not configured correctly
**Solution:**
- Verify `credentials: true` in backend CORS config
- Ensure frontend URL in `trustedOrigins` array
- Check browser Network tab for preflight OPTIONS requests

---

**Error:** "Session not persisting after refresh"
**Cause:** Cookies not being set/sent properly
**Solution:**
- Verify `credentials: true` in CORS
- Check cookie domain matches frontend domain
- In production, ensure HTTPS (cookies require Secure flag)
- Check browser DevTools → Application → Cookies

---

**Error:** "Additional fields not saving to database"
**Cause:** Schema or Better Auth config mismatch
**Solution:**
- Verify all fields in database schema match Better Auth `additionalFields`
- Ensure `input: true` is set for each additional field
- Check field types match (string, boolean, jsonb)
- Re-run database migrations

---

**Error:** "Cannot find module 'better-auth'"
**Cause:** Dependency not installed
**Solution:**
```bash
# In backend
cd auth-backend
npm install better-auth

# In frontend
cd docusaurus
npm install better-auth @better-auth/react
```

---

**Error:** "Database connection timeout"
**Cause:** Invalid DATABASE_URL or network issue
**Solution:**
- Verify Neon database connection string is correct
- Check `?sslmode=require` parameter is included
- Ensure Neon database is not paused
- Test connection with `psql` or database GUI

---

**Error:** "Type 'module' not supported" in Node
**Cause:** CommonJS vs ESM conflict
**Solution:**
- Ensure `"type": "module"` in package.json
- Use `.js` extensions in imports
- Or remove `"type": "module"` and use CommonJS (require/exports)

---

**Error:** "Passwords not matching" even when they match
**Cause:** Whitespace or form validation logic error
**Solution:**
```typescript
// Trim values before comparison
if (formData.password.trim() !== formData.confirmPassword.trim()) {
  setError('Passwords do not match');
  return;
}
```

---

**Error:** "Modal not closing after successful signup"
**Cause:** State not updated or page not refreshed
**Solution:**
```typescript
// After successful signup
onClose(); // Close modal
window.location.reload(); // Refresh to show authenticated state
```

---

### 8.2 Debugging Tips

**Enable Debug Logging:**

Backend (`src/index.ts`):
```typescript
app.use((req, res, next) => {
  console.log(`${req.method} ${req.path}`);
  next();
});
```

Frontend (auth-client.ts):
```typescript
// Check session state
const { data: session, isPending, error } = useSession();
console.log('Session:', session);
console.log('Is Pending:', isPending);
console.log('Error:', error);
```

**Check Network Requests:**
- Open browser DevTools → Network tab
- Filter by "Fetch/XHR"
- Look for calls to `/api/auth/*`
- Check request headers (cookies included?)
- Check response status codes and bodies

**Database Queries:**
```sql
-- Check if user exists
SELECT * FROM users WHERE email = 'user@example.com';

-- Check active sessions
SELECT s.*, u.email
FROM sessions s
JOIN users u ON s.user_id = u.id
WHERE s.expires_at > NOW();

-- Verify background data
SELECT
  email,
  software_background,
  hardware_background,
  robotics_experience
FROM users;
```

---

## 9. Future Enhancements

### 9.1 Content Personalization (Bonus Points)

**Implementation Plan:**
1. Access user background data in Docusaurus components:
   ```typescript
   import { useSession } from '@site/src/lib/auth-client';

   export default function PersonalizedContent() {
     const { data: session } = useSession();
     const user = session?.user;

     if (user?.softwareBackground === 'beginner') {
       return <BeginnerExplanation />;
     } else if (user?.softwareBackground === 'advanced') {
       return <AdvancedExplanation />;
     }

     return <DefaultExplanation />;
   }
   ```

2. Create difficulty-level badges:
   ```typescript
   // Show/hide content based on experience
   {user?.aiMlExperience === 'advanced' && (
     <AdvancedSection>...</AdvancedSection>
   )}
   ```

3. Customize code examples:
   ```typescript
   const codeExample = user?.programmingLanguages?.includes('Python')
     ? pythonCode
     : user?.programmingLanguages?.includes('C++')
     ? cppCode
     : genericCode;
   ```

### 9.2 OAuth Providers

Add Google/GitHub authentication:

```typescript
// In auth-backend/src/lib/auth.ts
export const auth = betterAuth({
  // ... existing config
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID,
      clientSecret: process.env.GITHUB_CLIENT_SECRET,
    },
  },
});
```

### 9.3 Email Verification

Enable in production:

```typescript
// In auth-backend/src/lib/auth.ts
export const auth = betterAuth({
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true, // Enable in production
    sendResetPasswordEmail: async ({ user, url }) => {
      // Send email using service (SendGrid, Resend, etc.)
    },
  },
});
```

### 9.4 User Profile Page

Create dedicated profile page:

```typescript
// docusaurus/src/pages/profile.tsx
export default function ProfilePage() {
  const { data: session } = useSession();

  return (
    <Layout>
      <h1>My Profile</h1>
      <div>
        <h2>Background Information</h2>
        <p>Software: {session?.user?.softwareBackground}</p>
        <p>Hardware: {session?.user?.hardwareBackground}</p>
        {/* ... display all fields ... */}

        <button onClick={handleUpdateProfile}>
          Update Profile
        </button>
      </div>
    </Layout>
  );
}
```

---

## 10. Appendices

### 10.1 Complete File Checklist

**Backend Files:**
- [ ] `auth-backend/package.json`
- [ ] `auth-backend/tsconfig.json`
- [ ] `auth-backend/drizzle.config.ts`
- [ ] `auth-backend/.env` (not committed)
- [ ] `auth-backend/.gitignore`
- [ ] `auth-backend/src/index.ts`
- [ ] `auth-backend/src/lib/auth.ts`
- [ ] `auth-backend/src/db/schema.ts`
- [ ] `auth-backend/src/db/index.ts`

**Frontend Files:**
- [ ] `docusaurus/.env.local` (not committed)
- [ ] `docusaurus/src/lib/auth-client.ts`
- [ ] `docusaurus/src/components/auth/SignupModal.tsx`
- [ ] `docusaurus/src/components/auth/LoginModal.tsx`
- [ ] `docusaurus/src/components/auth/AuthButtons.tsx`
- [ ] `docusaurus/src/components/auth/AuthModal.module.css`
- [ ] `docusaurus/src/components/auth/AuthButtons.module.css`
- [ ] `docusaurus/src/theme/Navbar/Content/index.tsx` (if swizzled)

### 10.2 Environment Variables Reference

**Development:**
```env
# Backend (.env)
DATABASE_URL=postgresql://...@...neon.tech/...?sslmode=require
BETTER_AUTH_SECRET=<32_char_random_string>
BETTER_AUTH_URL=http://localhost:5000
FRONTEND_URL=http://localhost:3000
NODE_ENV=development
PORT=5000

# Frontend (.env.local)
REACT_APP_AUTH_URL=http://localhost:5000
```

**Production:**
```env
# Backend (.env.production)
DATABASE_URL=postgresql://...@...neon.tech/...?sslmode=require
BETTER_AUTH_SECRET=<different_32_char_random_string>
BETTER_AUTH_URL=https://your-backend.railway.app
FRONTEND_URL=https://your-frontend.vercel.app
NODE_ENV=production
PORT=5000

# Frontend (Vercel Environment Variables)
REACT_APP_AUTH_URL=https://your-backend.railway.app
```

### 10.3 Useful Commands

```bash
# Backend
npm run dev           # Start development server
npm run build         # Build TypeScript
npm start             # Run production server
npm run db:generate   # Generate migrations
npm run db:migrate    # Apply migrations

# Frontend
npm start             # Start Docusaurus dev server
npm run build         # Build for production
npm run serve         # Serve production build locally

# Database
psql $DATABASE_URL    # Connect to database
```

### 10.4 API Endpoints Reference

**Better Auth Endpoints (Auto-generated):**
- `POST /api/auth/sign-up/email` - User registration
- `POST /api/auth/sign-in/email` - User login
- `POST /api/auth/sign-out` - User logout
- `GET /api/auth/session` - Get current session
- `POST /api/auth/reset-password` - Password reset (if enabled)
- `POST /api/auth/verify-email` - Email verification (if enabled)

**Custom Endpoints (Optional):**
- `GET /api/user/profile` - Get user profile
- `PATCH /api/user/profile` - Update user profile
- `PATCH /api/user/background` - Update background information

### 10.5 Dependencies Version Reference

**Backend:**
```json
{
  "dependencies": {
    "better-auth": "^1.0.0",
    "express": "^4.18.2",
    "cors": "^2.8.5",
    "drizzle-orm": "^0.33.0",
    "postgres": "^3.4.3",
    "dotenv": "^16.4.5"
  },
  "devDependencies": {
    "@types/express": "^4.17.21",
    "@types/cors": "^2.8.17",
    "@types/node": "^20.11.0",
    "tsx": "^4.7.0",
    "typescript": "^5.3.3",
    "drizzle-kit": "^0.24.0"
  }
}
```

**Frontend:**
```json
{
  "dependencies": {
    "better-auth": "^1.0.0",
    "@better-auth/react": "^1.0.0",
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "react": "^18.2.0",
    "react-dom": "^18.2.0"
  }
}
```

---

## 11. Success Validation

### 11.1 Pre-Submission Checklist

**Functionality:**
- [ ] User can sign up with all 12 background fields
- [ ] User data correctly stored in database
- [ ] User can log in with email/password
- [ ] Session persists across page refreshes
- [ ] User can log out successfully
- [ ] Auth state correctly shown in navbar
- [ ] Modal forms have proper validation
- [ ] Error messages display correctly

**Code Quality:**
- [ ] No TypeScript errors
- [ ] No console errors in browser
- [ ] Code follows consistent formatting
- [ ] All environment variables documented
- [ ] .env files in .gitignore

**Security:**
- [ ] Passwords hashed, not stored plain text
- [ ] Session cookies are HTTP-only
- [ ] CORS properly configured
- [ ] No sensitive data in frontend code
- [ ] Rate limiting on auth endpoints (production)

**UI/UX:**
- [ ] Responsive design works on mobile
- [ ] Forms are keyboard accessible
- [ ] Loading states work correctly
- [ ] Smooth animations and transitions
- [ ] Matches Docusaurus theme styling

**Documentation:**
- [ ] README with setup instructions
- [ ] Environment variables documented
- [ ] API endpoints documented
- [ ] Deployment guide included

### 11.2 Demo Script

**For Judges/Reviewers:**

1. **Show Signup Flow:**
   - Click "Sign Up" button in navbar
   - Fill Step 1 (Account Info)
   - Progress to Step 2 (Background Info)
   - Complete all fields
   - Submit and show successful login

2. **Show Database:**
   - Query database to show user record
   - Display all 12 background fields stored correctly

3. **Show Session Persistence:**
   - Refresh page → still logged in
   - Navigate between pages → session persists
   - Close and reopen browser → still logged in

4. **Show Logout:**
   - Click user dropdown
   - Click logout
   - Show auth buttons reappear

5. **Show Login:**
   - Click "Login" button
   - Enter credentials
   - Show successful login

---

## 12. Contact & Support

**Documentation:**
- Better Auth Docs: https://www.better-auth.com/docs
- Drizzle ORM: https://orm.drizzle.team/
- Docusaurus: https://docusaurus.io/

**Community:**
- Better Auth Discord: https://discord.gg/better-auth
- Stack Overflow: Tag `better-auth`

---

**Document Version:** 1.0.0
**Last Updated:** December 2025
**Maintained By:** Physical AI Robotics Textbook Team

## Response snapshot

I have successfully created the Better Auth Implementation Specification in the requested location. Here's what was completed:

1. Created the specification file at `C:\new\specs\3-auth-implementation\spec.md` containing the complete Better Auth Implementation Specification for the Physical AI & Robotics Textbook Authentication System.

2. The specification includes:
   - Executive Summary
   - System Architecture
   - Backend Specification (with environment config, database schema, Better Auth config, Express server, package config, and Drizzle config)
   - Frontend Specification (with auth client setup, component specifications, and styling)
   - Implementation Workflow
   - Testing Procedures
   - Deployment Guide
   - Troubleshooting Guide
   - Future Enhancements
   - Appendices

3. Created a validation checklist at `C:\new\specs\3-auth-implementation\checklists\requirements.md` to ensure specification quality.

The specification is comprehensive and ready for the next phase of development. It follows all the requirements specified in the original request, including the technology stack (Docusaurus + Better Auth), database (Neon PostgreSQL), and the collection of user background information for personalization purposes.

## Outcome

- ✅ Impact: Created comprehensive Better Auth implementation specification with all required components
- 🧪 Tests: null
- 📁 Files: Created spec.md and requirements.md checklist
- 🔁 Next prompts: Ready for planning phase (/sp.plan)
- 🧠 Reflection: The specification provides a complete guide for implementing the authentication system with user background collection

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Begin planning the implementation based on this specification