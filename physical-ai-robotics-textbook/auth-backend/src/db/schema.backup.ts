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
  
  // ADDED: Required for email/password authentication
  password: text('password'),

  // Custom background fields for personalization
  softwareBackground: text('software_background', {
    enum: ['beginner', 'intermediate', 'advanced', 'expert']
  }),

  hardwareBackground: text('hardware_background', {
    enum: ['none', 'basic', 'intermediate', 'advanced']
  }),

  programmingLanguages: jsonb('programming_languages').$type<string[]>(),

  roboticsExperience: text('robotics_experience', {
    enum: ['none', 'hobbyist', 'academic', 'professional']
  }),

  aiMlExperience: text('ai_ml_experience', {
    enum: ['none', 'basic', 'intermediate', 'advanced']
  }),

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
  createdAt: timestamp('created_at').notNull().defaultNow(),
  updatedAt: timestamp('updated_at').notNull().defaultNow(),
});

export const accounts = pgTable('accounts', {
  id: text('id').primaryKey(),
  userId: text('user_id').notNull().references(() => users.id, { onDelete: 'cascade' }),
  accountId: text('account_id').notNull(),
  providerId: text('provider_id').notNull(),
  accessToken: text('access_token'),
  refreshToken: text('refresh_token'),
  expiresAt: timestamp('expires_at'),
  createdAt: timestamp('created_at').notNull().defaultNow(),
  updatedAt: timestamp('updated_at').notNull().defaultNow(),
});

// ADDED: Verification tokens table (optional but recommended)
export const verificationTokens = pgTable('verification_tokens', {
  id: text('id').primaryKey(),
  identifier: text('identifier').notNull(),
  token: text('token').notNull().unique(),
  expiresAt: timestamp('expires_at').notNull(),
  createdAt: timestamp('created_at').notNull().defaultNow(),
});