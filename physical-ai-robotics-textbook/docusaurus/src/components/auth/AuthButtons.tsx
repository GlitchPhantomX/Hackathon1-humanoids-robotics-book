import React, { useState, useEffect, useCallback } from 'react';
import styles from './Auth.module.css';
import { authClient } from '../../lib/auth-client';
import LoginModal from './LoginModal';
import SignupModal from './SignupModal';
import ProfileDropdown from './ProfileDropdown';
import { AuthButtonsProps } from './types';
import ReactDOM from 'react-dom';

const AuthButtonsComponent: React.FC<AuthButtonsProps> = ({ onAuthChange }) => {
  const [session, setSession] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [showLoginModal, setShowLoginModal] = useState(false);
  const [showSignupModal, setShowSignupModal] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);

  /**
   * Fetch session via authClient only
   * Enhanced with better error handling and retry logic
   */
  const fetchSession = useCallback(async (retryCount = 0) => {
    try {
      const result = await authClient.getSession();
      console.log('AuthClient session:', result);

      if (result?.data?.session) {
        setSession(result.data.session);
        setLoading(false);
      } else if (result?.data?.user) {
        setSession(result.data);
        setLoading(false);
      } else {
        if (retryCount < 2) {
          setTimeout(() => fetchSession(retryCount + 1), 400);
        } else {
          setSession(null);
          setLoading(false);
        }
      }
    } catch (err) {
      console.log('Session fetch failed (backend may be unavailable):', err);
      // Don't retry indefinitely if backend is down
      setSession(null);
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    // Add a small delay to avoid race conditions
    const timer = setTimeout(() => {
      fetchSession();
    }, 100);

    return () => clearTimeout(timer);
  }, [fetchSession]);

  const handleLoginSuccess = useCallback(async () => {
    setShowLoginModal(false);
    setLoading(true);
    await new Promise((r) => setTimeout(r, 400));
    await fetchSession();
    if (onAuthChange) onAuthChange();
  }, [fetchSession, onAuthChange]);

  const handleSignupSuccess = useCallback(async () => {
    setShowSignupModal(false);
    setLoading(true);
    await new Promise((r) => setTimeout(r, 400));
    await fetchSession();
    if (onAuthChange) onAuthChange();
  }, [fetchSession, onAuthChange]);

  const handleSwitchToSignup = () => {
    setShowLoginModal(false);
    setShowSignupModal(true);
  };

  const handleSwitchToLogin = () => {
    setShowSignupModal(false);
    setShowLoginModal(true);
  };

  // Loading skeleton - show for shorter time to avoid flash
  if (loading) {
    return (
      <div className={styles.authContainer}>
        <div className={`${styles.authButton} ${styles.loading}`} style={{ opacity: 0.6 }}>
          <span style={{ fontSize: '12px' }}>⏳</span>
        </div>
      </div>
    );
  }

  // Authenticated user
  if (session?.user) {
    const userName = session.user.name || session.user.email || 'User';
    const userInitial = userName.charAt(0).toUpperCase();

    return (
      <div className={styles.authContainer}>
        <button
          className={`${styles.authButton} ${styles.navbarUserAvatar}`}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-label={`User menu for ${userName}`}
        >
          {userInitial}
        </button>

        {showDropdown &&
          ReactDOM.createPortal(
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
                  console.error('Logout failed:', err);
                  // Still clear session locally even if backend fails
                  setSession(null);
                  setShowDropdown(false);
                }
              }}
            />,
            document.body
          )}
      </div>
    );
  }

  // Not authenticated
  return (
    <div className={`${styles.authContainer} flex gap-3 mr-3`}>
      <button
        className={`${styles.authButton} ${styles.authButtonSecondary}`}
        onClick={() => setShowLoginModal(true)}
      >
        Login
      </button>

      <button
        className={styles.authButton}
        onClick={() => setShowSignupModal(true)}
      >
        Sign Up
      </button>

      <LoginModal
        isOpen={showLoginModal}
        onClose={() => setShowLoginModal(false)}
        onLoginSuccess={handleLoginSuccess}
        onSwitchToSignup={handleSwitchToSignup}
      />

      <SignupModal
        isOpen={showSignupModal}
        onClose={() => setShowSignupModal(false)}
        onSignupSuccess={handleSignupSuccess}
        onSwitchToLogin={handleSwitchToLogin}
      />
    </div>
  );
};

export default React.memo(AuthButtonsComponent);