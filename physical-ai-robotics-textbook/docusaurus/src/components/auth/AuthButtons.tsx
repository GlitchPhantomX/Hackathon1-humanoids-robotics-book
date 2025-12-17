import React, { useState, useEffect, useCallback } from 'react';
import styles from './Auth.module.css';
import LoginModal from './LoginModal';
import SignupModal from './SignupModal';
import ProfileDropdown from './ProfileDropdown';
import { AuthButtonsProps } from './types';
import ReactDOM from 'react-dom';
import { authClient } from '../../lib/auth-client';

const AuthButtonsComponent: React.FC<AuthButtonsProps> = ({ onAuthChange }) => {
  const [session, setSession] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [showLoginModal, setShowLoginModal] = useState(false);
  const [showSignupModal, setShowSignupModal] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);

  /**
   * Fetch session directly from backend API
   */
  const fetchSession = useCallback(async (retryCount = 0) => {
    try {
      // Force fresh fetch - no cache
      const result = await authClient.getSession();
      console.log('Session fetched:', result);

      if (result?.data?.session && result?.data?.user) {
        setSession({ 
          session: result.data.session, 
          user: result.data.user 
        });
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
      console.log('Session fetch failed:', err);
      setSession(null);
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    const timer = setTimeout(() => {
      fetchSession();
    }, 100);

    return () => clearTimeout(timer);
  }, [fetchSession]);

  // Listen for profile updates
  useEffect(() => {
    const handleProfileUpdate = () => {
      console.log('Profile updated event received, refreshing session...');
      setLoading(true);
      fetchSession();
    };

    // Listen for custom event
    window.addEventListener('profileUpdated', handleProfileUpdate);

    // Listen for storage event (cross-tab updates)
    window.addEventListener('storage', handleProfileUpdate);

    return () => {
      window.removeEventListener('profileUpdated', handleProfileUpdate);
      window.removeEventListener('storage', handleProfileUpdate);
    };
  }, [fetchSession]);

  const handleLoginSuccess = useCallback(async () => {
    setShowLoginModal(false);
    setLoading(true);
    await new Promise((r) => setTimeout(r, 500));
    await fetchSession();
    if (onAuthChange) onAuthChange();
  }, [fetchSession, onAuthChange]);

  const handleSignupSuccess = useCallback(async () => {
    setShowSignupModal(false);
    setLoading(true);
    await new Promise((r) => setTimeout(r, 500));
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

  const handleLogout = async () => {
    try {
      await authClient.signOut();
      setSession(null);
      setShowDropdown(false);
      if (onAuthChange) onAuthChange();
      window.location.href = '/';
    } catch (err) {
      console.error('Logout error:', err);
      setSession(null);
      setShowDropdown(false);
    }
  };

  // Loading skeleton
  if (loading) {
    return (
      <div className={styles.authContainer}>
        <div className={styles.authButton} style={{ opacity: 0.6, cursor: 'default' }}>
          <span className={styles.spinner} style={{ margin: 0 }}></span>
        </div>
      </div>
    );
  }

  // Authenticated user - PROFILE ICON
  if (session?.user) {
    const userName = session.user.name || session.user.email || 'User';
    const userInitial = userName.charAt(0).toUpperCase();
    const userImage = session.user.image;

    return (
      <div className={styles.authContainer}>
        <button
          className={styles.navbarUserAvatar}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-label={`User menu for ${userName}`}
          title={userName}
        >
          {userImage ? (
            <img 
              src={userImage} 
              alt={userName}
              style={{
                width: '100%',
                height: '100%',
                objectFit: 'cover',
                borderRadius: '50%',
              }}
            />
          ) : (
            userInitial
          )}
        </button>

        {showDropdown &&
          ReactDOM.createPortal(
            <ProfileDropdown
              user={session.user}
              isOpen={showDropdown}
              onClose={() => setShowDropdown(false)}
              onLogout={handleLogout}
            />,
            document.body
          )}
      </div>
    );
  }

  // Not authenticated - LOGIN & SIGNUP BUTTONS
  return (
    <div className={styles.authContainer}>
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