import React, { useState, useEffect, useCallback, useRef } from 'react';
import ReactDOM from 'react-dom';
import styles from './Auth.module.css';
import { authClient, announceToScreenReader } from '../../lib/auth-client';
import { FormData, SignupModalProps } from './types';

const SignupModal: React.FC<SignupModalProps> = ({ isOpen, onClose, onSignupSuccess }) => {
  const [step, setStep] = useState(1);
  const [formData, setFormData] = useState<FormData>({
    name: '',
    email: '',
    password: '',
    confirmPassword: '',
    softwareBackground: '',
    hardwareBackground: '',
    programmingLanguages: '',
    roboticsExperience: '',
    aiMlExperience: '',
    hasRosExperience: false,
    hasGpuAccess: false,
    learningGoals: '',
  });
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
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

  // Calculate password strength
  const calculatePasswordStrength = (password: string): { level: 'weak' | 'medium' | 'strong', label: string } => {
    if (password.length === 0) {
      return { level: 'weak', label: '' };
    }

    const hasMinLength = password.length >= 8;
    const hasUppercase = /[A-Z]/.test(password);
    const hasLowercase = /[a-z]/.test(password);
    const hasNumbers = /\d/.test(password);
    const hasSpecialChar = /[!@#$%^&*(),.?":{}|<>]/.test(password);

    const criteriaCount = [hasMinLength, hasUppercase, hasLowercase, hasNumbers, hasSpecialChar].filter(Boolean).length;

    if (criteriaCount >= 4 && password.length >= 8) {
      return { level: 'strong', label: 'Strong' };
    } else if (criteriaCount >= 2) {
      return { level: 'medium', label: 'Medium' };
    } else {
      return { level: 'weak', label: 'Weak' };
    }
  };

  const handleChange = useCallback((e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value, type } = e.target;
    const checked = (e.target as HTMLInputElement).checked;

    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
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

  const handleNext = useCallback(() => {
    // Validate current step
    const newErrors: Record<string, string> = {};

    if (step === 1) {
      if (!formData.name.trim()) {
        newErrors.name = 'Name is required';
      }
      if (!formData.email.trim()) {
        newErrors.email = 'Email is required';
      } else if (!/\S+@\S+\.\S+/.test(formData.email)) {
        newErrors.email = 'Email is invalid';
      }
      if (!formData.password) {
        newErrors.password = 'Password is required';
      } else if (formData.password.length < 8) {
        newErrors.password = 'Password must be at least 8 characters';
      }
      if (formData.password !== formData.confirmPassword) {
        newErrors.confirmPassword = 'Passwords do not match';
      }
    } else if (step === 2) {
      if (!formData.softwareBackground) {
        newErrors.softwareBackground = 'Software background is required';
      }
      if (!formData.hardwareBackground) {
        newErrors.hardwareBackground = 'Hardware background is required';
      }
      if (!formData.roboticsExperience) {
        newErrors.roboticsExperience = 'Robotics experience is required';
      }
      if (!formData.aiMlExperience) {
        newErrors.aiMlExperience = 'AI/ML experience is required';
      }
    }

    if (Object.keys(newErrors).length > 0) {
      setErrors(newErrors);
      return;
    }

    setErrors({});
    setStep(2);
  }, [step, formData]);

  const handlePrevious = useCallback(() => {
    setStep(1);
    setErrors({});
  }, []);

  const handleSubmit = useCallback(async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setErrors({});

    try {
      // FIXED: Send ALL fields in one signup request
      const signUpResult = await authClient.signUp.email({
        name: formData.name,
        email: formData.email,
        password: formData.password,
        // Custom fields - send as callbackURL data instead
        callbackURL: '/dashboard',
      });

      if (signUpResult.error) {
        // Provide more user-friendly error messages
        let errorMessage = 'Signup failed';
        const errMsg = signUpResult.error.message || '';

        if (errMsg.toLowerCase().includes('email')) {
          errorMessage = 'An account with this email already exists. Please try logging in instead.';
        } else if (errMsg.toLowerCase().includes('password')) {
          errorMessage = 'Password does not meet requirements. Please use at least 8 characters.';
        } else if (errMsg.toLowerCase().includes('network') || errMsg.toLowerCase().includes('timeout')) {
          errorMessage = 'Network error. Please check your connection and try again.';
        } else {
          errorMessage = 'An error occurred during signup. Please try again.';
        }
        throw new Error(errorMessage);
      }

      // After successful signup, update profile with custom fields
      if (signUpResult.data) {
        try {
          await fetch('http://localhost:5000/api/user/profile', {
            method: 'PATCH',
            headers: {
              'Content-Type': 'application/json',
            },
            credentials: 'include',
            body: JSON.stringify({
              softwareBackground: formData.softwareBackground,
              hardwareBackground: formData.hardwareBackground,
              programmingLanguages: formData.programmingLanguages,
              roboticsExperience: formData.roboticsExperience,
              aiMlExperience: formData.aiMlExperience,
              hasRosExperience: formData.hasRosExperience,
              hasGpuAccess: formData.hasGpuAccess,
              learningGoals: formData.learningGoals,
            }),
          });
        } catch (profileError) {
          console.warn('Profile update failed:', profileError);
          // Continue anyway - user is created
        }
      }

      // On success
      onSignupSuccess();
      onClose();
      announceToScreenReader('Account created successfully! Welcome to our platform.');
    } catch (error: any) {
      console.error('Signup error:', error);
      const errorMessage = error.message || 'An error occurred during signup. Please try again.';
      setErrors({ submit: errorMessage });
      announceToScreenReader(`Signup failed: ${errorMessage}`);
    } finally {
      setLoading(false);
    }
  }, [formData, onSignupSuccess, onClose]);

  if (!isOpen) return null;

  return ReactDOM.createPortal(
    <div
      className={styles.modalOverlay}
      role="dialog"
      aria-modal="true"
      aria-labelledby="signup-modal-title"
      onClick={onClose} // Close on backdrop click
    >
      <div
        className={styles.modalContent}
        ref={modalRef}
        onClick={(e) => e.stopPropagation()} // Prevent closing when clicking inside modal
      >
        <div className={styles.authForm}>
          <h2 id="signup-modal-title" className={styles.authFormHeader}>Create Account</h2>

          {/* Step indicator */}
          <div style={{ display: 'flex', justifyContent: 'center', marginBottom: '1.5rem' }} role="tablist" aria-label="Signup progress">
            <div
              style={{
                width: '30px',
                height: '30px',
                borderRadius: '50%',
                backgroundColor: step === 1 ? '#3498db' : '#bdc3c7',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: 'white',
                fontWeight: 'bold',
                marginRight: '10px'
              }}
              role="tab"
              aria-selected={step === 1}
              aria-label={`Step 1: Account Information ${step === 1 ? '(current)' : ''}`}
            >
              1
            </div>
            <div
              style={{
                width: '30px',
                height: '30px',
                borderRadius: '50%',
                backgroundColor: step === 2 ? '#3498db' : '#bdc3c7',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: 'white',
                fontWeight: 'bold'
              }}
              role="tab"
              aria-selected={step === 2}
              aria-label={`Step 2: Personalization ${step === 2 ? '(current)' : ''}`}
            >
              2
            </div>
          </div>

          <form onSubmit={handleSubmit}>
            {step === 1 && (
              <div role="tabpanel" aria-label="Account Information">
                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="name">
                    Full Name
                    {errors.name && <span className={styles.visuallyHidden}> Error: {errors.name}</span>}
                  </label>
                  <input
                    ref={firstInputRef}
                    type="text"
                    id="name"
                    name="name"
                    value={formData.name}
                    onChange={handleChange}
                    className={`${styles.authInput} ${errors.name ? styles.error : ''}`}
                    placeholder="Enter your full name"
                    aria-invalid={errors.name ? 'true' : 'false'}
                    aria-describedby={errors.name ? 'name-error' : undefined}
                    required
                  />
                  {errors.name && (
                    <div id="name-error" className={styles.authErrorMessage} role="alert">
                      {errors.name}
                    </div>
                  )}
                </div>

                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="email">
                    Email
                    {errors.email && <span className={styles.visuallyHidden}> Error: {errors.email}</span>}
                  </label>
                  <input
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
                    Password (minimum 8 characters)
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
                      placeholder="Create a password"
                      aria-invalid={errors.password ? 'true' : 'false'}
                      aria-describedby={errors.password ? 'password-error' : undefined}
                      required
                      minLength={8}
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
                  {/* Password strength indicator */}
                  <div className={styles.passwordStrengthContainer}>
                    <div className={styles.passwordStrengthBar}>
                      <div className={`${styles.passwordStrengthLevel} ${styles[`passwordStrength-${calculatePasswordStrength(formData.password).level}`]}`}></div>
                    </div>
                    <div className={styles.passwordStrengthLabel}>
                      {calculatePasswordStrength(formData.password).label}
                    </div>
                  </div>
                </div>

                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="confirmPassword">
                    Confirm Password
                    {errors.confirmPassword && <span className={styles.visuallyHidden}> Error: {errors.confirmPassword}</span>}
                  </label>
                  <div style={{ position: 'relative' }}>
                    <input
                      type={showConfirmPassword ? 'text' : 'password'}
                      id="confirmPassword"
                      name="confirmPassword"
                      value={formData.confirmPassword}
                      onChange={handleChange}
                      className={`${styles.authInput} ${errors.confirmPassword ? styles.error : ''}`}
                      placeholder="Confirm your password"
                      aria-invalid={errors.confirmPassword ? 'true' : 'false'}
                      aria-describedby={errors.confirmPassword ? 'confirmPassword-error' : undefined}
                      required
                    />
                    <button
                      type="button"
                      className={styles.eyeIcon}
                      onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                      aria-label={showConfirmPassword ? 'Hide password' : 'Show password'}
                    >
                      {showConfirmPassword ? '👁️' : '👁️‍🗨️'}
                    </button>
                  </div>
                  {errors.confirmPassword && (
                    <div id="confirmPassword-error" className={styles.authErrorMessage} role="alert">
                      {errors.confirmPassword}
                    </div>
                  )}
                </div>
              </div>
            )}

            {step === 2 && (
              <div className={styles.personalizationForm} role="tabpanel" aria-label="Personalization">
                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="softwareBackground">
                    Software Background
                    {errors.softwareBackground && <span className={styles.visuallyHidden}> Error: {errors.softwareBackground}</span>}
                  </label>
                  <select
                    id="softwareBackground"
                    name="softwareBackground"
                    value={formData.softwareBackground}
                    onChange={handleChange}
                    className={`${styles.authInput} ${errors.softwareBackground ? styles.error : ''}`}
                    aria-invalid={errors.softwareBackground ? 'true' : 'false'}
                    aria-describedby={errors.softwareBackground ? 'softwareBackground-error' : undefined}
                    required
                  >
                    <option value="">Select your software background</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                    <option value="expert">Expert</option>
                  </select>
                  {errors.softwareBackground && (
                    <div id="softwareBackground-error" className={styles.authErrorMessage} role="alert">
                      {errors.softwareBackground}
                    </div>
                  )}
                </div>

                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="hardwareBackground">
                    Hardware Background
                    {errors.hardwareBackground && <span className={styles.visuallyHidden}> Error: {errors.hardwareBackground}</span>}
                  </label>
                  <select
                    id="hardwareBackground"
                    name="hardwareBackground"
                    value={formData.hardwareBackground}
                    onChange={handleChange}
                    className={`${styles.authInput} ${errors.hardwareBackground ? styles.error : ''}`}
                    aria-invalid={errors.hardwareBackground ? 'true' : 'false'}
                    aria-describedby={errors.hardwareBackground ? 'hardwareBackground-error' : undefined}
                    required
                  >
                    <option value="">Select your hardware background</option>
                    <option value="none">No experience</option>
                    <option value="basic">Basic</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                  {errors.hardwareBackground && (
                    <div id="hardwareBackground-error" className={styles.authErrorMessage} role="alert">
                      {errors.hardwareBackground}
                    </div>
                  )}
                </div>

                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="programmingLanguages">
                    Programming Languages
                  </label>
                  <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
                    {['Python', 'JavaScript', 'C++', 'Java', 'ROS', 'C#', 'TypeScript', 'Go', 'Rust', 'MATLAB'].map((lang) => (
                      <div key={lang} style={{ display: 'flex', alignItems: 'center' }}>
                        <input
                          type="checkbox"
                          id={`lang-${lang}`}
                          name="programmingLanguages"
                          value={lang}
                          checked={formData.programmingLanguages.includes(lang)}
                          onChange={(e) => {
                            const currentLanguages = formData.programmingLanguages ? formData.programmingLanguages.split(',').map(s => s.trim()) : [];
                            let newLanguages: string[];

                            if (e.target.checked) {
                              newLanguages = [...currentLanguages, lang];
                            } else {
                              newLanguages = currentLanguages.filter(l => l !== lang);
                            }

                            setFormData(prev => ({
                              ...prev,
                              programmingLanguages: newLanguages.join(',')
                            }));
                          }}
                          style={{ marginRight: '8px' }}
                        />
                        <label htmlFor={`lang-${lang}`}>{lang}</label>
                      </div>
                    ))}
                  </div>
                </div>

                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="roboticsExperience">
                    Robotics Experience
                    {errors.roboticsExperience && <span className={styles.visuallyHidden}> Error: {errors.roboticsExperience}</span>}
                  </label>
                  <select
                    id="roboticsExperience"
                    name="roboticsExperience"
                    value={formData.roboticsExperience}
                    onChange={handleChange}
                    className={`${styles.authInput} ${errors.roboticsExperience ? styles.error : ''}`}
                    aria-invalid={errors.roboticsExperience ? 'true' : 'false'}
                    aria-describedby={errors.roboticsExperience ? 'roboticsExperience-error' : undefined}
                    required
                  >
                    <option value="">Select your robotics experience</option>
                    <option value="none">No experience</option>
                    <option value="hobbyist">Hobbyist</option>
                    <option value="academic">Academic</option>
                    <option value="professional">Professional</option>
                  </select>
                  {errors.roboticsExperience && (
                    <div id="roboticsExperience-error" className={styles.authErrorMessage} role="alert">
                      {errors.roboticsExperience}
                    </div>
                  )}
                </div>

                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="aiMlExperience">
                    AI/ML Experience
                    {errors.aiMlExperience && <span className={styles.visuallyHidden}> Error: {errors.aiMlExperience}</span>}
                  </label>
                  <select
                    id="aiMlExperience"
                    name="aiMlExperience"
                    value={formData.aiMlExperience}
                    onChange={handleChange}
                    className={`${styles.authInput} ${errors.aiMlExperience ? styles.error : ''}`}
                    aria-invalid={errors.aiMlExperience ? 'true' : 'false'}
                    aria-describedby={errors.aiMlExperience ? 'aiMlExperience-error' : undefined}
                    required
                  >
                    <option value="">Select your AI/ML experience</option>
                    <option value="none">No experience</option>
                    <option value="basic">Basic</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                  {errors.aiMlExperience && (
                    <div id="aiMlExperience-error" className={styles.authErrorMessage} role="alert">
                      {errors.aiMlExperience}
                    </div>
                  )}
                </div>

                <div className={styles.fieldGroup}>
                  <div className={styles.checkboxGroup} style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                    <input
                      type="checkbox"
                      id="hasRosExperience"
                      name="hasRosExperience"
                      checked={formData.hasRosExperience}
                      onChange={handleChange}
                      className={styles.checkboxInput}
                      aria-describedby="hasRosExperience-help"
                    />
                    <label htmlFor="hasRosExperience">Do you have ROS experience?</label>
                  </div>
                  <div id="hasRosExperience-help" className={styles.fieldHelp} style={{ fontSize: '0.8rem', color: '#6c757d', marginTop: '0.25rem' }}>
                    ROS (Robot Operating System) experience
                  </div>
                </div>

                <div className={styles.fieldGroup}>
                  <div className={styles.checkboxGroup} style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                    <input
                      type="checkbox"
                      id="hasGpuAccess"
                      name="hasGpuAccess"
                      checked={formData.hasGpuAccess}
                      onChange={handleChange}
                      className={styles.checkboxInput}
                      aria-describedby="hasGpuAccess-help"
                    />
                    <label htmlFor="hasGpuAccess">Do you have access to a GPU?</label>
                  </div>
                  <div id="hasGpuAccess-help" className={styles.fieldHelp} style={{ fontSize: '0.8rem', color: '#6c757d', marginTop: '0.25rem' }}>
                    Access to a graphics processing unit for AI/ML tasks
                  </div>
                </div>

                <div className={styles.fieldGroup}>
                  <label className={styles.authLabel} htmlFor="learningGoals">
                    Learning Goals
                  </label>
                  <textarea
                    id="learningGoals"
                    name="learningGoals"
                    value={formData.learningGoals}
                    onChange={handleChange}
                    className={`${styles.authInput} ${styles.mbSmall}`}
                    placeholder="What are your learning goals?"
                    aria-describedby="learningGoals-help"
                    maxLength={1000}
                  />
                  <div className={styles.characterCount}>
                    {formData.learningGoals.length}/1000
                  </div>
                  <div id="learningGoals-help" className={styles.fieldHelp} style={{ fontSize: '0.8rem', color: '#6c757d', marginTop: '0.25rem' }}>
                    Describe your learning goals for this course
                  </div>
                </div>
              </div>
            )}

            {errors.submit && (
              <div className={styles.authErrorMessage} role="alert">
                {errors.submit}
              </div>
            )}

            <div style={{ display: 'flex', gap: '10px', marginTop: '1rem' }}>
              {step > 1 && (
                <button
                  type="button"
                  onClick={handlePrevious}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter' || e.key === ' ') {
                      e.preventDefault();
                      handlePrevious();
                    }
                  }}
                  className={styles.authButton}
                  style={{ backgroundColor: '#95a5a6' }}
                  disabled={loading}
                  aria-label="Go to previous step"
                >
                  Previous
                </button>
              )}
              {step < 2 ? (
                <button
                  type="button"
                  onClick={handleNext}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter' || e.key === ' ') {
                      e.preventDefault();
                      handleNext();
                    }
                  }}
                  className={styles.authButton}
                  disabled={loading}
                  aria-label="Go to next step"
                >
                  Next
                </button>
              ) : (
                <button
                  type="submit"
                  className={styles.authButton}
                  disabled={loading}
                  aria-label="Create your account"
                  aria-busy={loading}
                >
                  {loading ? (
                    <span>
                      <span className={styles.spinner} aria-hidden="true"></span> Signing up...
                    </span>
                  ) : (
                    'Create Account'
                  )}
                </button>
              )}
            </div>
          </form>

          <div className={styles.authFormFooter}>
            <div className={styles.textCenter}>
              Already have an account?{' '}
              <button
                type="button"
                onClick={onClose}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' || e.key === ' ') {
                    e.preventDefault();
                    onClose();
                  }
                }}
                className={styles.authLink}
                aria-label="Switch to sign in form"
              >
                Sign in
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>,
    document.body
  );
};

export default SignupModal;