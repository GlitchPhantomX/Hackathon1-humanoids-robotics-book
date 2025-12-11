# Data Model: Better Auth Implementation

## 1. Database Schema

### 1.1 Users Table
The users table extends Better Auth's default user schema with additional fields for personalization:

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

### 1.2 Sessions Table
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

### 1.3 Accounts Table
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

## 2. Entity Relationships

### 2.1 User-Session Relationship
- One user to many sessions (one-to-many)
- Sessions are cascaded when user is deleted
- Foreign key constraint ensures referential integrity

### 2.2 User-Account Relationship
- One user to many accounts (one-to-many)
- Accounts are cascaded when user is deleted
- Supports multiple authentication providers per user

## 3. Field Validation Rules

### 3.1 Required Fields
- `email`: Must be valid email format, unique across all users
- `name`: Minimum 2 characters
- `softwareBackground`: Enum value from ['beginner', 'intermediate', 'advanced', 'expert']
- `hardwareBackground`: Enum value from ['none', 'basic', 'intermediate', 'advanced']
- `roboticsExperience`: Enum value from ['none', 'hobbyist', 'academic', 'professional']
- `aiMlExperience`: Enum value from ['none', 'basic', 'intermediate', 'advanced']

### 3.2 Optional Fields
- `image`: User profile image URL
- `programmingLanguages`: Array of programming languages as JSON
- `learningGoals`: Text field up to 500 characters
- `hasRosExperience`: Boolean flag for ROS experience
- `hasGpuAccess`: Boolean flag for GPU access

### 3.3 System Fields
- `id`: Unique identifier (auto-generated)
- `emailVerified`: Boolean flag for email verification status
- `createdAt`: Timestamp when user was created
- `updatedAt`: Timestamp when user was last updated
- `expiresAt`: Session expiration time

## 4. Indexing Strategy

### 4.1 Primary Indexes
- `users.id`: Primary key index
- `users.email`: Unique index for fast lookups
- `sessions.id`: Primary key index
- `sessions.token`: Unique index for secure token lookup
- `accounts.id`: Primary key index

### 4.2 Foreign Key Indexes
- `sessions.user_id`: Index for user-session relationship queries
- `accounts.user_id`: Index for user-account relationship queries

### 4.3 Performance Indexes (Future)
- `users.created_at`: For time-based queries
- Potential composite indexes for complex queries

## 5. Data Types Mapping

### 5.1 PostgreSQL to TypeScript
- `text` → `string`
- `boolean` → `boolean`
- `timestamp` → `Date`
- `jsonb` → `any` (with type casting to specific types)

### 5.2 Enum Values
- `softwareBackground`: 'beginner' | 'intermediate' | 'advanced' | 'expert'
- `hardwareBackground`: 'none' | 'basic' | 'intermediate' | 'advanced'
- `roboticsExperience`: 'none' | 'hobbyist' | 'academic' | 'professional'
- `aiMlExperience`: 'none' | 'basic' | 'intermediate' | 'advanced'

## 6. Security Considerations

### 6.1 Data Protection
- Passwords are hashed and not stored in the users table (handled by Better Auth)
- Personal data is stored with appropriate access controls
- Session tokens are stored securely and expire appropriately

### 6.2 Privacy
- User personalization data is used only for content customization
- No sensitive personal information is collected beyond what's needed for personalization
- Data retention policies should be established for user data

## 7. Migration Considerations

### 7.1 Schema Evolution
- New custom fields can be added with appropriate default values
- Enum values can be extended without breaking existing data
- Field types should not be changed without proper migration strategy

### 7.2 Backward Compatibility
- Default values ensure new fields don't break existing users
- Optional fields allow gradual rollout of new features
- Enum additions are backward compatible