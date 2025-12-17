import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from '../db/index.js';
import * as schema from '../db/schema.js';

const isProduction = process.env.NODE_ENV === 'production';

const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
    schema: {
      user: schema.users,
      session: schema.sessions,
      account: schema.accounts,
      verificationToken: schema.verificationTokens,
    }
  }),
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    minPasswordLength: 8,
    maxPasswordLength: 128,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7,
    updateAge: 60 * 60 * 24,
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60,
    },
    // Cookie options for cross-origin
    cookie: {
      name: 'better-auth.session_token',
      sameSite: 'none',  // Cross-origin ke liye
      secure: true,      // HTTPS required
      httpOnly: true,
    },
  },
  user: {
    additionalFields: {
      softwareBackground: { type: 'string', required: false },
      hardwareBackground: { type: 'string', required: false },
      programmingLanguages: { type: 'string', required: false },
      roboticsExperience: { type: 'string', required: false },
      aiMlExperience: { type: 'string', required: false },
      hasRosExperience: { type: 'boolean', required: false },
      hasGpuAccess: { type: 'boolean', required: false },
      learningGoals: { type: 'string', required: false },
    }
  },
  trustedOrigins: [
    'http://localhost:3000',
    'http://localhost:3001',
    'https://hackathon1-humanoids-robotics-book.vercel.app',
  ],
  baseURL: isProduction 
    ? 'https://hackathon1-humanoids-robotics-book-production.up.railway.app'
    : 'http://localhost:5000',
  basePath: '/api/auth',
  secret: process.env.BETTER_AUTH_SECRET || 'fallback-secret-key-min-32-chars',
});

export default auth;
export type Session = typeof auth.$Infer.Session;