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
        const errorMessages = Object.values(newErrors).join(', ');
        announceToScreenReader(`Login form errors: ${errorMessages}`);
        return;
      }

      console.log('Attempting login with Better Auth client...');

      // Use Better Auth client (handles cookies properly)
      const loginResult = await authClient.signIn.email({
        email: formData.email,
        password: formData.password,
      });

      console.log('Login result:', loginResult);

      if (loginResult.error) {
        console.error('Login error:', loginResult.error);
        
        let errorMessage = 'Invalid email or password. Please try again.';
        
        // Handle specific error messages
        if (loginResult.error.message) {
          if (loginResult.error.message.includes('Invalid email')) {
            errorMessage = 'Invalid email or password.';
          } else if (loginResult.error.message.includes('User not found')) {
            errorMessage = 'No account found with this email.';
          } else {
            errorMessage = loginResult.error.message;
          }
        }
        
        throw new Error(errorMessage);
      }

      if (!loginResult.data) {
        throw new Error('Login completed but no data returned.');
      }

      console.log('✅ Login successful!');
      announceToScreenReader('Login successful. Welcome back!');
      
      // Call success callback if provided
      if (onLoginSuccess) {
        onLoginSuccess(loginResult.data);
      }
      
      // Close modal
      onClose();
      
      // Clear cache and reload to refresh auth state
      localStorage.clear();
      window.location.href = '/?refresh=' + Date.now();
      
    } catch (error: any) {
      console.error('❌ Login error:', error);
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
    onClick={onClose}
    style={{
      backgroundColor: 'rgba(0, 0, 0, 0.6)',
      zIndex: 10000, // Higher z-index to appear above hero section
    }}
  >
    <div
      className={styles.modalContent}
      ref={modalRef}
      onClick={(e) => e.stopPropagation()}
      style={{
        backgroundColor: 'var(--color-bg)',
        color: 'var(--color-text)',
        border: '1px solid var(--color-modal-border)',
      }}
    >
      <div className={styles.authForm}>
        <h2 
          id="login-modal-title" 
          className={styles.authFormHeader}
          style={{ color: 'var(--color-text)', fontSize: '1.5rem', fontWeight: '600' }}
        >
          Sign In
        </h2>

        <form onSubmit={handleSubmit}>
          <div className={styles.fieldGroup}>
            <label 
              className={styles.authLabel} 
              htmlFor="email"
              style={{ color: 'var(--color-text)' }}
            >
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
              disabled={loading}
              style={{
                backgroundColor: 'var(--color-physical-from-CTA-background)',
                color: 'var(--color-text)',
                border: `1px solid ${errors.email ? '#dc2626' : 'var(--color-input-border)'}`,
                transition: 'border-color 0.2s',
                opacity: loading ? 0.7 : 1,
              }}
              onFocus={(e) => {
                if (!errors.email) {
                  e.target.style.borderColor = 'var(--color-input-border-focus)';
                }
              }}
              onBlur={(e) => {
                if (!errors.email) {
                  e.target.style.borderColor = 'var(--color-input-border)';
                }
              }}
            />
            {errors.email && (
              <div id="email-error" className={styles.authErrorMessage} role="alert">
                {errors.email}
              </div>
            )}
          </div>

          <div className={styles.fieldGroup}>
            <label 
              className={styles.authLabel} 
              htmlFor="password"
              style={{ color: 'var(--color-text)' }}
            >
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
                disabled={loading}
                style={{
                  backgroundColor: 'var(--color-physical-from-CTA-background)',
                  color: 'var(--color-text)',
                  border: `1px solid ${errors.password ? '#dc2626' : 'var(--color-input-border)'}`,
                  transition: 'border-color 0.2s',
                  opacity: loading ? 0.7 : 1,
                }}
                onFocus={(e) => {
                  if (!errors.password) {
                    e.target.style.borderColor = 'var(--color-input-border-focus)';
                  }
                }}
                onBlur={(e) => {
                  if (!errors.password) {
                    e.target.style.borderColor = 'var(--color-input-border)';
                  }
                }}
              />
              <button
                type="button"
                className={styles.eyeIcon}
                onClick={() => setShowPassword(!showPassword)}
                aria-label={showPassword ? 'Hide password' : 'Show password'}
                disabled={loading}
                style={{
                  color: 'var(--color-muted)',
                  opacity: loading ? 0.5 : 1,
                }}
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
            style={{
              backgroundColor: loading ? 'var(--color-hover-button-background)' : 'var(--color-dark-button-background)',
              color: 'white',
              opacity: loading ? 0.7 : 1,
            }}
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
          <div 
            className={styles.textCenter}
            style={{ color: 'var(--color-text)' }}
          >
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
              disabled={loading}
              style={{
                color: 'var(--color-accent)',
                background: 'none',
                border: 'none',
                textDecoration: 'underline',
                cursor: 'pointer',
                fontWeight: '600',
                opacity: loading ? 0.5 : 1,
              }}
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