// TypeScript interfaces for authentication components

export interface User {
  id: string;
  email: string;
  name: string;
  image?: string;
  createdAt: string;
  updatedAt: string;
  softwareBackground: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  hardwareBackground: 'none' | 'basic' | 'intermediate' | 'advanced';
  programmingLanguages?: string; // JSON string
  roboticsExperience: 'none' | 'hobbyist' | 'academic' | 'professional';
  aiMlExperience: 'none' | 'basic' | 'intermediate' | 'advanced';
  hasRosExperience: boolean;
  hasGpuAccess: boolean;
  learningGoals?: string;
}

export interface Session {
  user: User;
  expiresAt: string;
}

export interface FormData {
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

export interface SignupModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSignupSuccess: () => Promise<void> | void;
  onSwitchToLogin: () => void; // ✅ ADD THIS
}

export interface LoginModalProps {
  isOpen: boolean;
  onClose: () => void;
  onLoginSuccess: () => void;
  onSwitchToSignup: () => void;
}

export interface AuthButtonsProps {
  client?: any; // Better Auth client instance
  onAuthChange?: () => void; // Callback when auth state changes
}

export interface ApiResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    message: string;
    code?: string;
  };
}

export interface UserProfileUpdateData {
  softwareBackground?: string;
  hardwareBackground?: string;
  programmingLanguages?: string;
  roboticsExperience?: string;
  aiMlExperience?: string;
  hasRosExperience?: boolean;
  hasGpuAccess?: boolean;
  learningGoals?: string;
}

export interface AuthResponse {
  user: User;
  session?: Session;
  error?: string;
}

// Accessibility-related types
export interface AccessibilityAttributes {
  'aria-label'?: string;
  'aria-labelledby'?: string;
  'aria-describedby'?: string;
  'aria-hidden'?: boolean;
  'aria-invalid'?: boolean;
  'aria-required'?: boolean;
  'aria-expanded'?: boolean;
  'aria-haspopup'?: boolean;
  'role'?: string;
}