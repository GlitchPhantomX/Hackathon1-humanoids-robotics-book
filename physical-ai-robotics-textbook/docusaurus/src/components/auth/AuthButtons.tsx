import React, { useState, useEffect, useCallback, useMemo } from 'react';
import ReactDOM from 'react-dom';
import styles from './Auth.module.css';
import { authClient } from '../../lib/auth-client';
import LoginModal from './LoginModal';
import SignupModal from './SignupModal';
import ProfileDropdown from './ProfileDropdown';
import { AuthButtonsProps } from './types';
import { useRef } from 'react';

const AuthButtonsComponent: React.FC<AuthButtonsProps> = ({ onAuthChange }) => {
  const [session, setSession] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [showLoginModal, setShowLoginModal] = useState(false);
  const [showSignupModal, setShowSignupModal] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);

  // Track authentication state change time for performance monitoring
  const lastAuthStateChange = useRef<number>(Date.now());

  // Fetch session manually with retry logic
  const fetchSession = useCallback(async (retryCount = 0) => {
    const startTime = Date.now();
    try {
      // Make a direct fetch call to see raw response
      const rawResponse = await fetch('http://localhost:5000/api/auth/get-session', {
        credentials: 'include',
        headers: {
          'Cache-Control': 'no-cache',
        },
      });
      const rawData = await rawResponse.json();
      console.log('Raw API response:', rawData); // Debug log
      
      const result = await authClient.getSession();
      console.log('AuthClient result:', result); // Debug log
      
      // Check if session data exists - better-auth returns {data: {...}, error: null}
      if (result?.data?.session) {
        console.log('Valid session found:', result.data);
        setSession(result.data);
        setLoading(false);
      } else if (result?.data?.user) {
        // Some auth libraries return user directly in data
        console.log('User found in data:', result.data);
        setSession(result.data);
        setLoading(false);
      } else {
        console.log('No session data found, result:', result);
        
        // Retry logic - sometimes session takes a moment to be available
        if (retryCount < 3) {
          console.log(`Retrying session fetch (attempt ${retryCount + 1})...`);
          setTimeout(() => fetchSession(retryCount + 1), 500);
        } else {
          setSession(null);
          setLoading(false);
        }
      }

      // Measure time for authentication state change to reflect in UI
      const authChangeTime = Date.now() - startTime;
      if (authChangeTime > 500) {
        console.warn(`Authentication state change took ${authChangeTime}ms to reflect in UI, exceeding 500ms threshold`);
      }
    } catch (err) {
      console.error('Failed to fetch session:', err);
      setSession(null);
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchSession();
  }, [fetchSession]);

  const handleLoginSuccess = useCallback(async () => {
    console.log('Login successful, fetching session...'); // Debug log
    setShowLoginModal(false);
    setLoading(true); // Show loading state
    
    // Add delays to ensure backend has processed the session
    await new Promise(resolve => setTimeout(resolve, 500));
    
    // Try multiple times with increasing delays
    for (let i = 0; i < 3; i++) {
      await fetchSession();
      await new Promise(resolve => setTimeout(resolve, 300));
    }
    
    if (onAuthChange) onAuthChange();
  }, [fetchSession, onAuthChange]);

  const handleSignupSuccess = useCallback(async () => {
    console.log('Signup successful, fetching session...'); // Debug log
    setShowSignupModal(false);
    setLoading(true); // Show loading state
    
    // Add delays to ensure backend has processed the session
    await new Promise(resolve => setTimeout(resolve, 500));
    
    // Try multiple times with increasing delays
    for (let i = 0; i < 3; i++) {
      await fetchSession();
      await new Promise(resolve => setTimeout(resolve, 300));
    }
    
    if (onAuthChange) onAuthChange();
  }, [fetchSession, onAuthChange]);

  const handleSwitchToSignup = useCallback(() => {
    setShowLoginModal(false);
    setShowSignupModal(true);
  }, []);

  const handleSwitchToLogin = useCallback(() => {
    setShowSignupModal(false);
    setShowLoginModal(true);
  }, []);

  // Loading state - show skeleton while checking session
  if (loading) {
    return (
      <div className={styles.authContainer}>
        <div className={`${styles.authButton} ${styles.loading}`}>
          Loading...
        </div>
      </div>
    );
  }

  // If user is authenticated, show profile avatar
  if (session?.user) {
    const userName = session.user.name || session.user.email || 'User';
    const userInitial = userName.charAt(0).toUpperCase();

    return (
      <div className={styles.authContainer}>
        <button
          className={`${styles.authButton} ${styles.navbarUserAvatar}`}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-label={`User menu for ${userName}`}
          aria-expanded={showDropdown}
        >
          {userInitial}
        </button>

        {/* Profile Dropdown */}
        {showDropdown && ReactDOM.createPortal(
          <div
            style={{
              position: 'fixed',
              zIndex: 9999,
              top: 'calc(100% + 8px)',
              right: '0',
              transform: 'translateY(0)',
            }}
            onClick={(e) => e.stopPropagation()}
          >
            <ProfileDropdown
              user={session.user}
              isOpen={showDropdown}
              onClose={() => setShowDropdown(false)}
              onLogout={async () => {
                try {
                  await authClient.signOut();
                  setSession(null);
                  setShowDropdown(false);
                  if (onAuthChange) onAuthChange();
                } catch (err) {
                  console.error('Logout error:', err);
                }
              }}
            />
          </div>,
          document.body
        )}
      </div>
    );
  }

  // If user is not authenticated, show login and signup buttons
  return (
    <div className={`${styles.authContainer} flex gap-3 mr-3`} >
      <button
        className={`${styles.authButton} ${styles.authButtonSecondary}`}
        onClick={() => setShowLoginModal(true)}
        aria-label="Sign in to your account"
      >
        Login
      </button>
      <button
        className={styles.authButton}
        onClick={() => setShowSignupModal(true)}
        aria-label="Create a new account"
      >
        Sign Up
      </button>

      {/* Login Modal */}
      <LoginModal
        isOpen={showLoginModal}
        onClose={() => setShowLoginModal(false)}
        onLoginSuccess={handleLoginSuccess}
        onSwitchToSignup={handleSwitchToSignup}
      />

      {/* Signup Modal */}
      <SignupModal
        isOpen={showSignupModal}
        onClose={() => setShowSignupModal(false)}
        onSignupSuccess={handleSignupSuccess}
        onSwitchToLogin={handleSwitchToLogin}
      />
    </div>
  );
};

const AuthButtons = React.memo(AuthButtonsComponent);
export default AuthButtons;