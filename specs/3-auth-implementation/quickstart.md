# Quickstart Guide: Better Auth Implementation

## 1. Prerequisites

### 1.1 System Requirements
- Node.js v18+ installed
- npm or yarn package manager
- Git for version control
- Access to Neon PostgreSQL database

### 1.2 Development Tools
- Code editor (VS Code recommended)
- Terminal/command line access
- Browser with developer tools
- Database client (optional, for verification)

## 2. Project Setup

### 2.1 Backend Setup

#### Step 1: Create Backend Directory
```bash
cd physical-ai-robotics-textbook
mkdir auth-backend
cd auth-backend
npm init -y
```

#### Step 2: Install Dependencies
```bash
# Core dependencies
npm install better-auth express cors drizzle-orm postgres dotenv

# Development dependencies
npm install -D @types/express @types/cors @types/node tsx typescript drizzle-kit
```

#### Step 3: Configure TypeScript
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

### 2.2 Database Setup

#### Step 1: Create Database Schema
Create `src/db/schema.ts`:
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

export const sessions = pgTable('sessions', {
  id: text('id').primaryKey(),
  userId: text('user_id').notNull().references(() => users.id, { onDelete: 'cascade' }),
  expiresAt: timestamp('expires_at').notNull(),
  token: text('token').notNull().unique(),
  ipAddress: text('ip_address'),
  userAgent: text('user_agent'),
});

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

#### Step 2: Database Connection
Create `src/db/index.ts`:
```typescript
import { drizzle } from 'drizzle-orm/postgres-js';
import postgres from 'postgres';
import * as schema from './schema';

const connectionString = process.env.DATABASE_URL!;

const client = postgres(connectionString);
export const db = drizzle(client, { schema });
```

### 2.3 Better Auth Configuration

#### Step 1: Configure Better Auth
Create `src/lib/auth.ts`:
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

#### Step 2: Create Express Server
Create `src/index.ts`:
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

### 2.4 Environment Configuration

#### Step 1: Create Environment File
Create `.env` in the auth-backend directory:
```env
# Database
DATABASE_URL=postgresql://user:password@host/database?sslmode=require

# Better Auth
BETTER_AUTH_SECRET=your_32_character_random_string_here
BETTER_AUTH_URL=http://localhost:5000
FRONTEND_URL=http://localhost:3000

# Server
NODE_ENV=development
PORT=5000
```

#### Step 2: Drizzle Configuration
Create `drizzle.config.ts`:
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

#### Step 3: Update Package.json Scripts
Update `package.json`:
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
  }
}
```

## 3. Frontend Setup

### 3.1 Install Frontend Dependencies
```bash
cd ../docusaurus
npm install better-auth @better-auth/react
```

### 3.2 Create Auth Client
Create `src/lib/auth-client.ts`:
```typescript
import { createAuthClient } from '@better-auth/react';

const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_URL || 'http://localhost:5000',
});

export const { signIn, signUp, signOut, useSession } = authClient;
export { authClient };
```

### 3.3 Add Environment Variable
Create `.env.local` in the docusaurus directory:
```
REACT_APP_AUTH_URL=http://localhost:5000
```

## 4. Running the Application

### 4.1 Backend
```bash
cd auth-backend
npm run dev
```

### 4.2 Frontend
```bash
cd docusaurus
npm start
```

## 5. Testing the Setup

### 5.1 Backend API Testing
Use curl or Postman to test the backend:

**Test Signup:**
```bash
curl -X POST http://localhost:5000/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
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
  }'
```

**Test Login:**
```bash
curl -X POST http://localhost:5000/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123"
  }'
```

### 5.2 Frontend Component Testing
1. Start both servers
2. Open the Docusaurus site in a browser
3. Verify auth buttons appear in the navbar
4. Test signup and login flows
5. Verify session persistence

## 6. Next Steps

### 6.1 Component Development
1. Create SignupModal component with 2-step form
2. Create LoginModal component
3. Create AuthButtons component for navbar
4. Add CSS modules for styling
5. Integrate with Docusaurus navbar

### 6.2 Database Migrations
1. Run `npm run db:generate` to create migration files
2. Run `npm run db:migrate` to apply migrations to the database
3. Verify tables are created in your Neon database

### 6.3 Production Deployment
1. Update environment variables for production
2. Deploy backend to Railway or similar platform
3. Update frontend to use production auth URL
4. Test complete authentication flow