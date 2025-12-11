// Shared utility functions for form validation and other common operations
import { useState, useEffect } from 'react';

// Modal animation constants
export const MODAL_ANIMATION_DURATION = 300; // ms
export const MODAL_ANIMATION_EASING = 'ease-out';
export const MODAL_SCALE_START = 0.95;
export const MODAL_SCALE_END = 1.0;

// Password validation utility
export const validatePassword = (password: string): { isValid: boolean; errors: string[] } => {
  const errors: string[] = [];

  if (password.length < 8) {
    errors.push('Password must be at least 8 characters long');
  }

  if (!/[A-Z]/.test(password)) {
    errors.push('Password must contain at least one uppercase letter');
  }

  if (!/[a-z]/.test(password)) {
    errors.push('Password must contain at least one lowercase letter');
  }

  if (!/\d/.test(password)) {
    errors.push('Password must contain at least one number');
  }

  if (!/[!@#$%^&*(),.?":{}|<>]/.test(password)) {
    errors.push('Password must contain at least one special character');
  }

  return {
    isValid: errors.length === 0,
    errors
  };
};

// Email validation utility
export const validateEmail = (email: string): { isValid: boolean; error?: string } => {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;

  if (!email) {
    return { isValid: false, error: 'Email is required' };
  }

  if (!emailRegex.test(email)) {
    return { isValid: false, error: 'Please enter a valid email address' };
  }

  return { isValid: true };
};

// Name validation utility
export const validateName = (name: string): { isValid: boolean; error?: string } => {
  if (!name) {
    return { isValid: false, error: 'Name is required' };
  }

  if (name.length < 1 || name.length > 100) {
    return { isValid: false, error: 'Name must be between 1 and 100 characters' };
  }

  return { isValid: true };
};

// Generic form field validation hook
export const useFormFieldValidation = <T>(
  initialValue: T,
  validator: (value: T) => { isValid: boolean; error?: string }
) => {
  const [value, setValue] = useState<T>(initialValue);
  const [error, setError] = useState<string | undefined>();
  const [touched, setTouched] = useState(false);

  const validate = (): boolean => {
    const result = validator(value);
    setError(result.error);
    return result.isValid;
  };

  const handleChange = (newValue: T) => {
    setValue(newValue);
    if (touched) {
      // Re-validate when user types after having touched the field
      validator(newValue).error
        ? setError(validator(newValue).error)
        : setError(undefined);
    }
  };

  const handleBlur = () => {
    setTouched(true);
    validate();
  };

  const reset = () => {
    setValue(initialValue);
    setError(undefined);
    setTouched(false);
  };

  return {
    value,
    error,
    touched,
    isValid: !error && touched,
    handleChange,
    handleBlur,
    reset,
    validate
  };
};

// Debounced validation hook to avoid excessive validation calls
export function useDebounce<T>(value: T, delay: number): T {
  const [debouncedValue, setDebouncedValue] = useState<T>(value);

  useEffect(() => {
    const handler = setTimeout(() => {
      setDebouncedValue(value);
    }, delay);

    return () => {
      clearTimeout(handler);
    };
  }, [value, delay]);

  return debouncedValue;
}

// Validation hook with debounce
export const useValidatedField = <T>(
  initialValue: T,
  validator: (value: T) => { isValid: boolean; error?: string },
  debounceDelay: number = 300
) => {
  const [value, setValue] = useState<T>(initialValue);
  const [error, setError] = useState<string | undefined>();
  const [touched, setTouched] = useState(false);
  const debouncedValue = useDebounce(value, debounceDelay);

  useEffect(() => {
    if (touched) {
      const result = validator(debouncedValue);
      setError(result.error);
    }
  }, [debouncedValue, touched, validator]);

  const validate = (): boolean => {
    const result = validator(value);
    setError(result.error);
    setTouched(true);
    return result.isValid;
  };

  const handleChange = (newValue: T) => {
    setValue(newValue);
  };

  const handleBlur = () => {
    setTouched(true);
    // Validate immediately on blur
    const result = validator(value);
    setError(result.error);
  };

  const reset = () => {
    setValue(initialValue);
    setError(undefined);
    setTouched(false);
  };

  return {
    value,
    error,
    touched,
    isValid: !error && value !== initialValue, // Consider valid if no error and changed from initial
    handleChange,
    handleBlur,
    reset,
    validate
  };
};

// Form submission preparation utility
export const prepareFormData = (formData: Record<string, any>): Record<string, any> => {
  const preparedData: Record<string, any> = {};

  Object.keys(formData).forEach(key => {
    const value = formData[key];

    // Trim strings to remove leading/trailing whitespace
    if (typeof value === 'string') {
      preparedData[key] = value.trim();
    } else {
      preparedData[key] = value;
    }
  });

  return preparedData;
};

// Validation utility for background profile fields
export const validateBackgroundProfile = (profile: {
  softwareBackground?: string;
  hardwareBackground?: string;
  programmingLanguages?: string[];
  roboticsExperience?: string;
  aiMlExperience?: string;
  hasRosExperience?: boolean;
  hasGpuAccess?: boolean;
  learningGoals?: string;
}): { isValid: boolean; errors: Record<string, string> } => {
  const errors: Record<string, string> = {};

  // Validate software background if provided
  if (profile.softwareBackground && !['beginner', 'intermediate', 'advanced', 'expert'].includes(profile.softwareBackground)) {
    errors.softwareBackground = 'Please select a valid software background level';
  }

  // Validate hardware background if provided
  if (profile.hardwareBackground && !['none', 'basic', 'intermediate', 'advanced'].includes(profile.hardwareBackground)) {
    errors.hardwareBackground = 'Please select a valid hardware background level';
  }

  // Validate robotics experience if provided
  if (profile.roboticsExperience && !['none', 'hobbyist', 'academic', 'professional'].includes(profile.roboticsExperience)) {
    errors.roboticsExperience = 'Please select a valid robotics experience level';
  }

  // Validate AI/ML experience if provided
  if (profile.aiMlExperience && !['none', 'basic', 'intermediate', 'advanced'].includes(profile.aiMlExperience)) {
    errors.aiMlExperience = 'Please select a valid AI/ML experience level';
  }

  // Validate learning goals length if provided
  if (profile.learningGoals && profile.learningGoals.length > 1000) {
    errors.learningGoals = 'Learning goals must be less than 1000 characters';
  }

  return {
    isValid: Object.keys(errors).length === 0,
    errors
  };
};