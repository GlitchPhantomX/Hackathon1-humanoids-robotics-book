import React, { useState, useEffect, useCallback, useRef } from 'react';
import ReactDOM from 'react-dom';
import styles from './Auth.module.css';
import { authClient, announceToScreenReader } from '../../lib/auth-client';
import { LoginModalProps } from './types';

interface FormData {
  email: string;
  password: string;
}

const LoginModal: React.FC<LoginModalProps> = ({ isOpen, onClose, onLoginSuccess, onSwitchToSignup }) => {
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
  });
  const [showPassword, setShowPassword] = useState(false);
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const modalRef = useRef<HTMLDivElement>(null);
  const firstInputRef = useRef<HTMLInputElement>(null);

  // Handle Escape key press
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  // Handle backdrop click
  useEffect(() => {
    if (!isOpen) return;

    const handleClickOutside = (e: MouseEvent) => {
      if (modalRef.current && !modalRef.current.contains(e.target as Node)) {
        onClose();
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [isOpen, onClose]);

  // Focus management
  useEffect(() => {
    if (isOpen && firstInputRef.current) {
      firstInputRef.current.focus();
    }
  }, [isOpen]);

  // Trap focus within modal when open
  useEffect(() => {
    if (!isOpen) return;

    const focusableElements = modalRef.current?.querySelectorAll(
      'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
    ) as NodeListOf<HTMLElement>;

    const firstElement = focusableElements[0];
    const lastElement = focusableElements[focusableElements.length - 1];

    const handleTabKey = (e: KeyboardEvent) => {
      if (e.key === 'Tab') {
        if (e.shiftKey) {
          if (document.activeElement === firstElement) {
            lastElement.focus();
            e.preventDefault();
          }
        } else {
          if (document.activeElement === lastElement) {
            firstElement.focus();
            e.preventDefault();
          }
        }
      }
    };

    document.addEventListener('keydown', handleTabKey);
    return () => document.removeEventListener('keydown', handleTabKey);
  }, [isOpen]);

  const handleChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    // Clear error when user starts typing
    if (errors[name]) {
      setErrors(prev => {
        const newErrors = { ...prev };
        delete newErrors[name];
        return newErrors;
      });
    }
  }, [errors]);

  const handleSubmit = useCallback(async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setErrors({});

    try {
      // Validate form
      const newErrors: Record<string, string> = {};
      if (!formData.email.trim()) {
        newErrors.email = 'Email is required';
      } else if (!/\S+@\S+\.\S+/.test(formData.email)) {
        newErrors.email = 'Email is invalid';
      }
      if (!formData.password) {
        newErrors.password = 'Password is required';
      }

      if (Object.keys(newErrors).length > 0) {
        setErrors(newErrors);
        setLoading(false);
        // Announce errors to screen readers
        const errorMessages = Object.values(newErrors).join(', ');
        announceToScreenReader(`Login form errors: ${errorMessages}`);
        return;
      }

      // Attempt login with Better Auth
      const loginResult = await authClient.signIn.email({
        email: formData.email,
        password: formData.password,
        callbackURL: '/', // Redirect to home after login
      });

      if (loginResult.error) {
        // Provide more user-friendly error messages
        let errorMessage = 'Login failed';
        if (loginResult.error.message?.includes('email') || loginResult.error.message?.includes('password')) {
          errorMessage = 'Invalid email or password. Please try again.';
        } else if (loginResult.error.message?.includes('verify')) {
          errorMessage = 'Please verify your email address before logging in.';
        } else if (loginResult.error.message?.includes('network') || loginResult.error.message?.includes('timeout')) {
          errorMessage = 'Network error. Please check your connection and try again.';
        } else {
          errorMessage = 'Invalid email or password. Please try again.';
        }
        throw new Error(errorMessage);
      }

      // On successful login
      onLoginSuccess();
      onClose();
      announceToScreenReader('Login successful. Welcome back!');
    } catch (error: any) {
      console.error('Login error:', error);
      const errorMessage = error.message || 'An error occurred during login. Please try again.';
      setErrors({ submit: errorMessage });
      announceToScreenReader(`Login failed: ${errorMessage}`);
    } finally {
      setLoading(false);
    }
  }, [formData, onLoginSuccess, onClose]);

  if (!isOpen) return null;

  return ReactDOM.createPortal(
    <div
      className={styles.modalOverlay}
      role="dialog"
      aria-modal="true"
      aria-labelledby="login-modal-title"
      onClick={onClose} // Close on backdrop click
    >
      <div
        className={styles.modalContent}
        ref={modalRef}
        onClick={(e) => e.stopPropagation()} // Prevent closing when clicking inside modal
      >
        <div className={styles.authForm}>
          <h2 id="login-modal-title" className={styles.authFormHeader}>
            Sign In
          </h2>

          <form onSubmit={handleSubmit}>
            <div className={styles.fieldGroup}>
              <label className={styles.authLabel} htmlFor="email">
                Email
                {errors.email && <span className={styles.visuallyHidden}> Error: {errors.email}</span>}
              </label>
              <input
                ref={firstInputRef}
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleChange}
                className={`${styles.authInput} ${errors.email ? styles.error : ''}`}
                placeholder="Enter your email"
                aria-invalid={errors.email ? 'true' : 'false'}
                aria-describedby={errors.email ? 'email-error' : undefined}
                required
              />
              {errors.email && (
                <div id="email-error" className={styles.authErrorMessage} role="alert">
                  {errors.email}
                </div>
              )}
            </div>

            <div className={styles.fieldGroup}>
              <label className={styles.authLabel} htmlFor="password">
                Password
                {errors.password && <span className={styles.visuallyHidden}> Error: {errors.password}</span>}
              </label>
              <div style={{ position: 'relative' }}>
                <input
                  type={showPassword ? 'text' : 'password'}
                  id="password"
                  name="password"
                  value={formData.password}
                  onChange={handleChange}
                  className={`${styles.authInput} ${errors.password ? styles.error : ''}`}
                  placeholder="Enter your password"
                  aria-invalid={errors.password ? 'true' : 'false'}
                  aria-describedby={errors.password ? 'password-error' : undefined}
                  required
                />
                <button
                  type="button"
                  className={styles.eyeIcon}
                  onClick={() => setShowPassword(!showPassword)}
                  aria-label={showPassword ? 'Hide password' : 'Show password'}
                >
                  {showPassword ? '👁️' : '👁️‍🗨️'}
                </button>
              </div>
              {errors.password && (
                <div id="password-error" className={styles.authErrorMessage} role="alert">
                  {errors.password}
                </div>
              )}
            </div>

            {errors.submit && (
              <div className={styles.authErrorMessage} role="alert">
                {errors.submit}
              </div>
            )}

            <button
              type="submit"
              className={styles.authButton}
              disabled={loading}
              aria-busy={loading}
            >
              {loading ? (
                <span>
                  <span className={styles.spinner} aria-hidden="true"></span> Signing in...
                </span>
              ) : (
                'Sign In'
              )}
            </button>
          </form>

          <div className={styles.authFormFooter}>
            <div className={styles.textCenter}>
              Don't have an account?{' '}
              <button
                type="button"
                onClick={onSwitchToSignup}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' || e.key === ' ') {
                    e.preventDefault();
                    onSwitchToSignup();
                  }
                }}
                className={styles.authLink}
                aria-label="Switch to sign up form"
              >
                Sign up
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>,
    document.body
  );
};

export default LoginModal;