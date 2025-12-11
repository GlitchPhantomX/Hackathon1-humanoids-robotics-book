import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from '../db';
import * as schema from '../db/schema';

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
    }
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
    process.env.FRONTEND_URL || '',
  ],
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:5000',
  secret: process.env.BETTER_AUTH_SECRET!,
});

export default auth;
export type Session = typeof auth.$Infer.Session;
