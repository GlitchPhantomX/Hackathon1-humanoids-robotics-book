import 'dotenv/config';
import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import rateLimit from 'express-rate-limit';
import { auth } from './lib/auth';
import { requestLogger, errorLogger } from './utils/logger';

const app = express();
const PORT = process.env.PORT || 5000;
const isProduction = process.env.NODE_ENV === 'production';

// -----------------------------
// Security middleware
app.use(
  helmet({
    contentSecurityPolicy: false, // disable for dev; enable in prod
    crossOriginEmbedderPolicy: false,
  })
);

// Rate limiting
const limiter = rateLimit({
  windowMs: 15 * 60 * 1000,
  max: 100,
  message: 'Too many requests from this IP, please try again later.',
  standardHeaders: true,
  legacyHeaders: false,
});
app.use(limiter);

// Additional security headers
app.use((req, res, next) => {
  res.setHeader('X-Content-Type-Options', 'nosniff');
  res.setHeader('X-Frame-Options', 'DENY');
  res.setHeader('X-XSS-Protection', '1; mode=block');
  res.setHeader('Referrer-Policy', 'strict-origin-when-cross-origin');
  next();
});

// -----------------------------
// JSON parsing & CORS
app.use(express.json());
app.use(
  cors({
    origin: isProduction
      ? process.env.FRONTEND_URL
      : process.env.FRONTEND_URL || 'http://localhost:3000',
    credentials: true,
    allowedHeaders: ['Content-Type', 'Authorization', 'Cache-Control', 'X-Requested-With'],
    exposedHeaders: ['Set-Cookie'],
  })
);

// -----------------------------
// Logging
app.use(requestLogger);
app.use(errorLogger);

// -----------------------------
// Auth routes using auth.api (no auth.handler)
app.post('/api/auth/sign-up/email', async (req, res) => {
  try {
    const { email, password } = req.body;
    if (!email || !password) return res.status(400).json({ message: 'Email & password required.' });

    const result = await auth.api.signUp({
      body: { email, password },
      headers: req.headers as any,
    });

    res.json(result);
  } catch (error) {
    console.error('Sign up error:', error);
    res.status(500).json({ error: 'Sign up failed', details: error });
  }
});

app.post('/api/auth/sign-in/email', async (req, res) => {
  try {
    const { email, password } = req.body;
    if (!email || !password) return res.status(400).json({ message: 'Email & password required.' });

    const result = await auth.api.signIn.email({
      body: { email, password },
      headers: req.headers as any,
    });

    res.json(result);
  } catch (error) {
    console.error('Sign in error:', error);
    res.status(500).json({ error: 'Sign in failed', details: error });
  }
});

app.post('/api/auth/sign-out', async (req, res) => {
  try {
    await auth.api.signOut({ headers: req.headers as any });
    res.json({ message: 'Logged out successfully' });
  } catch (error) {
    console.error('Sign out error:', error);
    res.status(500).json({ error: 'Sign out failed', details: error });
  }
});

app.get('/api/auth/get-session', async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers as any });
    if (!session) return res.status(401).json({ error: 'No active session' });
    res.json(session);
  } catch (error) {
    console.error('Get session error:', error);
    res.status(500).json({ error: 'Failed to fetch session', details: error });
  }
});

// -----------------------------
// User profile endpoints
app.get('/api/user/profile', async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers as any });
    if (!session) return res.status(401).json({ error: 'Unauthorized' });

    res.json({ user: session.user });
  } catch (error) {
    console.error('Profile fetch error:', error);
    res.status(500).json({ error: 'Failed to fetch profile' });
  }
});

app.patch('/api/user/profile', async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers as any });
    if (!session) return res.status(401).json({ error: 'Unauthorized' });

    const updatedUser = await auth.api.updateUser({
      body: { id: session.user.id, ...req.body },
      headers: req.headers as any,
    });

    res.json({ user: updatedUser.user || updatedUser });
  } catch (error) {
    console.error('Profile update error:', error);
    res.status(500).json({ error: 'Failed to update profile' });
  }
});

app.put('/api/user/profile', async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers as any });
    if (!session) return res.status(401).json({ error: 'Unauthorized' });

    const updatedUser = await auth.api.updateUser({
      body: { id: session.user.id, ...req.body },
      headers: req.headers as any,
    });

    res.json({ user: updatedUser.user || updatedUser });
  } catch (error) {
    console.error('Profile update error:', error);
    res.status(500).json({ error: 'Failed to update profile' });
  }
});

// -----------------------------
// Root route
app.get('/', (_req, res) => {
  res.send('Backend server is running!');
});

// -----------------------------
// Start server
app.listen(PORT, () => {
  console.log(`🔥 Auth server running on http://localhost:${PORT}`);
});
