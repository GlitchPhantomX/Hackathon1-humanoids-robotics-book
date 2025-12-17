// TypeScript interfaces for authentication components

// ============================================
// USER & SESSION TYPES
// ============================================

export interface User {
  id: string;
  createdAt: Date;
  updatedAt: Date;
  email: string;
  emailVerified: boolean;
  name: string;
  image?: string | null;
  
  // Custom profile fields
  softwareBackground?: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  hardwareBackground?: 'none' | 'basic' | 'intermediate' | 'advanced';
  programmingLanguages?: string; // Comma-separated string or JSON
  roboticsExperience?: 'none' | 'hobbyist' | 'academic' | 'professional';
  aiMlExperience?: 'none' | 'basic' | 'intermediate' | 'advanced';
  hasRosExperience?: boolean;
  hasGpuAccess?: boolean;
  learningGoals?: string;
}

export interface Session {
  session: {
    id: string;
    userId: string;
    expiresAt: Date;
  };
  user: User;
}

// ============================================
// FORM DATA TYPES
// ============================================

export interface SignupFormData {
  // Step 1: Account Information
  name: string;
  email: string;
  password: string;
  confirmPassword: string;

  // Step 2: Background Information
  softwareBackground: string;
  hardwareBackground: string;
  programmingLanguages: string;
  roboticsExperience: string;
  aiMlExperience: string;
  hasRosExperience: boolean;
  hasGpuAccess: boolean;
  learningGoals: string;
}

export interface LoginFormData {
  email: string;
  password: string;
}

export interface ProfileFormData {
  name: string;
  email: string;
  softwareBackground: string;
  hardwareBackground: string;
  programmingLanguages: string;
  roboticsExperience: string;
  aiMlExperience: string;
  hasRosExperience: boolean;
  hasGpuAccess: boolean;
  learningGoals: string;
}

// Backward compatibility alias
export type FormData = SignupFormData;

// ============================================
// COMPONENT PROPS TYPES
// ============================================

export interface AuthButtonsProps {
  client?: any; // Better Auth client instance (optional)
  onAuthChange?: () => void; // Callback when auth state changes
}

export interface LoginModalProps {
  isOpen: boolean;
  onClose: () => void;
  onLoginSuccess: () => void;
  onSwitchToSignup: () => void;
}

export interface SignupModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSignupSuccess: () => Promise<void> | void;
  onSwitchToLogin: () => void;
}

export interface ProfileDropdownProps {
  user: User;
  isOpen: boolean;
  onClose: () => void;
  onLogout: () => void;
}

// ============================================
// API RESPONSE TYPES
// ============================================

export interface ApiResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    message: string;
    code?: string;
  };
}

export interface AuthResponse {
  user?: User;
  session?: Session;
  token?: string;
  error?: string;
}

export interface UserProfileUpdateData {
  name?: string;
  softwareBackground?: string;
  hardwareBackground?: string;
  programmingLanguages?: string;
  roboticsExperience?: string;
  aiMlExperience?: string;
  hasRosExperience?: boolean;
  hasGpuAccess?: boolean;
  learningGoals?: string;
}

// ============================================
// ACCESSIBILITY TYPES
// ============================================

export interface AccessibilityAttributes {
  'aria-label'?: string;
  'aria-labelledby'?: string;
  'aria-describedby'?: string;
  'aria-hidden'?: boolean;
  'aria-invalid'?: boolean;
  'aria-required'?: boolean;
  'aria-expanded'?: boolean;
  'aria-haspopup'?: boolean;
  'aria-live'?: 'off' | 'polite' | 'assertive';
  'aria-atomic'?: boolean;
  'role'?: string;
}

// ============================================
// UTILITY TYPES
// ============================================

export type ExperienceLevel = 'none' | 'beginner' | 'basic' | 'intermediate' | 'advanced' | 'expert' | 'professional' | 'hobbyist' | 'academic';

export type LoadingState = 'idle' | 'loading' | 'success' | 'error';

export interface ErrorState {
  message: string;
  field?: string;
  code?: string;
}

export interface ValidationError {
  [key: string]: string;
}

// ============================================
// AUTH CLIENT TYPES
// ============================================

export interface AuthClient {
  getSession: () => Promise<{ data: Session | null; error: any }>;
  signIn: {
    email: (data: { email: string; password: string }) => Promise<AuthResponse>;
  };
  signUp: {
    email: (data: { email: string; password: string; name: string }) => Promise<AuthResponse>;
  };
  signOut: () => Promise<{ data: any; error: any }>;
}

// ============================================
// BETTER AUTH SPECIFIC TYPES
// ============================================

export interface BetterAuthSession {
  data: {
    session: {
      id: string;
      userId: string;
      expiresAt: Date;
    } | null;
    user: User | null;
  };
  error: any;
}

export interface BetterAuthResponse<T = any> {
  data: T | null;
  error: {
    message: string;
    status?: number;
  } | null;
}