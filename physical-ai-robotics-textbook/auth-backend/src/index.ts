import 'dotenv/config';
import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import rateLimit from 'express-rate-limit';
import cookieParser from 'cookie-parser';
import { toNodeHandler } from 'better-auth/node';
import auth from './lib/auth.js';
import { requestLogger, errorLogger } from './utils/logger.js';

const app = express();
const PORT = process.env.PORT || 5000;
const isProduction = process.env.NODE_ENV === 'production';

// ✅ CRITICAL: Trust proxy for Railway - MUST be before other middleware
app.set('trust proxy', 1);

// Security middleware
app.use(
  helmet({
    contentSecurityPolicy: false,
    crossOriginEmbedderPolicy: false,
  })
);

// Rate limiting - trust proxy is handled by app.set above
const limiter = rateLimit({
  windowMs: 15 * 60 * 1000,
  max: 100,
  message: 'Too many requests from this IP, please try again later.',
  standardHeaders: true,
  legacyHeaders: false,
  // trust proxy handled by app.set('trust proxy', 1) above
});
app.use(limiter);

// Parsing middleware
app.use(cookieParser());
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// CORS Configuration
app.use(
  cors({
    origin: [
      'http://localhost:3000',
      'https://hackathon1-humanoids-robotics-book.vercel.app'
    ],
    credentials: true,
    allowedHeaders: ['Content-Type', 'Authorization', 'Cookie'],
    exposedHeaders: ['Set-Cookie'],
    methods: ['GET', 'POST', 'PUT', 'PATCH', 'DELETE', 'OPTIONS'],
  })
);

app.options('*', cors());

// Logging
app.use(requestLogger);

// Better Auth Handler
app.all('/api/auth/*', toNodeHandler(auth));

// User profile endpoints
app.get('/api/user/profile', async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers as any,
    });
    
    if (!session || !session.session) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    return res.status(200).json({ user: session.user });
    
  } catch (error) {
    console.error('Profile fetch error:', error);
    return res.status(500).json({ error: 'Failed to fetch profile' });
  }
});

app.patch('/api/user/profile', async (req, res) => {
  try {
    console.log('📝 Profile update request received');
    console.log('Request body:', req.body);
    console.log('Cookies:', req.cookies);

    const session = await auth.api.getSession({
      headers: req.headers as any,
    });

    console.log('Session:', session);

    if (!session || !session.session) {
      console.log('❌ No session found');
      return res.status(401).json({ error: 'Unauthorized' });
    }

    console.log('✅ User authenticated:', session.user.id);

    const {
      name,
      softwareBackground,
      hardwareBackground,
      programmingLanguages,
      roboticsExperience,
      aiMlExperience,
      hasRosExperience,
      hasGpuAccess,
      learningGoals
    } = req.body;

    const { db } = await import('./db/index.js');
    const { users } = await import('./db/schema.js');
    const { eq } = await import('drizzle-orm');

    const updateData: any = {};

    if (name !== undefined) updateData.name = name;
    if (softwareBackground !== undefined) updateData.softwareBackground = softwareBackground;
    if (hardwareBackground !== undefined) updateData.hardwareBackground = hardwareBackground;
    if (programmingLanguages !== undefined) updateData.programmingLanguages = programmingLanguages;
    if (roboticsExperience !== undefined) updateData.roboticsExperience = roboticsExperience;
    if (aiMlExperience !== undefined) updateData.aiMlExperience = aiMlExperience;
    if (hasRosExperience !== undefined) updateData.hasRosExperience = hasRosExperience;
    if (hasGpuAccess !== undefined) updateData.hasGpuAccess = hasGpuAccess;
    if (learningGoals !== undefined) updateData.learningGoals = learningGoals;

    console.log('Update data:', updateData);

    const [updatedUser] = await db
      .update(users)
      .set(updateData)
      .where(eq(users.id, session.user.id))
      .returning();

    console.log('✅ Profile updated successfully');

    return res.status(200).json({
      success: true,
      user: updatedUser
    });

  } catch (error: any) {
    console.error('❌ Profile update error:', error);
    console.error('Error stack:', error.stack);
    return res.status(500).json({
      error: 'Failed to update profile',
      message: error.message
    });
  }
});

// Language preference endpoints
app.get('/api/user/preferences', async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers as any,
    });

    if (!session || !session.session) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const { db } = await import('./db/index.js');
    const { users } = await import('./db/schema.js');
    const { eq } = await import('drizzle-orm');

    const [user] = await db
      .select({
        languagePreference: users.languagePreference
      })
      .from(users)
      .where(eq(users.id, session.user.id));

    if (!user) {
      return res.status(404).json({ error: 'User not found' });
    }

    return res.status(200).json({
      language: user.languagePreference,
      theme: 'dark', // Default theme
      notifications: true, // Default notifications
      lastUpdated: new Date().toISOString()
    });

  } catch (error) {
    console.error('User preferences fetch error:', error);
    return res.status(500).json({ error: 'Failed to fetch user preferences' });
  }
});

app.post('/api/user/preferences', async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers as any,
    });

    if (!session || !session.session) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const { language } = req.body;

    // Validate language
    const validLanguages = ['en', 'ur', 'hi'];
    if (!language || !validLanguages.includes(language)) {
      return res.status(400).json({ error: 'Invalid language preference' });
    }

    const { db } = await import('./db/index.js');
    const { users } = await import('./db/schema.js');
    const { eq } = await import('drizzle-orm');

    const [updatedUser] = await db
      .update(users)
      .set({ languagePreference: language })
      .where(eq(users.id, session.user.id))
      .returning();

    if (!updatedUser) {
      return res.status(404).json({ error: 'User not found' });
    }

    return res.status(200).json({
      success: true,
      preferences: {
        language: updatedUser.languagePreference,
        theme: 'dark', // Default theme
        notifications: true, // Default notifications
        lastUpdated: new Date().toISOString()
      }
    });

  } catch (error) {
    console.error('User preferences update error:', error);
    return res.status(500).json({ error: 'Failed to update user preferences' });
  }
});

app.put('/api/user/profile', async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers as any,
    });
    
    if (!session || !session.session) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const updatedUser = await auth.api.updateUser({
      body: req.body,
      headers: req.headers as any,
    });

    return res.status(200).json({ user: updatedUser });
    
  } catch (error) {
    console.error('Profile update error:', error);
    return res.status(500).json({ error: 'Failed to update profile' });
  }
});

// Error logging
app.use(errorLogger);

// Root route
app.get('/', (_req, res) => {
  res.send('Backend server is running!');
});

// Start server
app.listen(PORT, () => {
  console.log(`🔥 Auth server running on port ${PORT}`);
  console.log(`📍 CORS enabled for: http://localhost:3000`);
});