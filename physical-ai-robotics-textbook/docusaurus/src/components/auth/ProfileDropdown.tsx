import React, { useEffect, useRef, memo } from "react";
import type { User } from "./types";

interface ProfileDropdownProps {
  user: User;
  isOpen: boolean;
  onClose: () => void;
  onLogout: () => void;
}

const ProfileDropdownComponent: React.FC<ProfileDropdownProps> = ({
  user,
  isOpen,
  onClose,
  onLogout,
}) => {
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Escape Key Close
  useEffect(() => {
    if (!isOpen) return;
    const handleKey = (e: KeyboardEvent) => e.key === "Escape" && onClose();
    document.addEventListener("keydown", handleKey);
    return () => document.removeEventListener("keydown", handleKey);
  }, [isOpen, onClose]);

  // Outside Click Close
  useEffect(() => {
    if (!isOpen) return;
    const handleClick = (e: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(e.target as Node)) {
        onClose();
      }
    };
    document.addEventListener("mousedown", handleClick);
    return () => document.removeEventListener("mousedown", handleClick);
  }, [isOpen, onClose]);

  if (!isOpen) return null;

  // Format Languages
  const formatProgrammingLanguages = (languages: string | undefined) => {
    if (!languages) return [];
    try {
      if (languages.startsWith('[')) {
        const parsed = JSON.parse(languages);
        return Array.isArray(parsed) ? parsed : [];
      }
      return languages.split(',').map(lang => lang.trim()).filter(Boolean);
    } catch {
      return languages.split(',').map(lang => lang.trim()).filter(Boolean);
    }
  };

  // Capitalize first letter
  const capitalize = (str: string | undefined) => {
    if (!str) return "Not specified";
    return str.charAt(0).toUpperCase() + str.slice(1);
  };

  const languages = formatProgrammingLanguages(user.programmingLanguages);

  return (
    <div
      ref={dropdownRef}
      style={{
        position: 'fixed',
        top: '70px',
        right: '20px',
        width: '380px',
        maxHeight: 'calc(100vh - 100px)',
        zIndex: 9999,
        backgroundColor: 'var(--color-bg)',
        borderRadius: '12px',
        boxShadow: '0 10px 40px rgba(0, 0, 0, 0.15)',
        border: '1px solid var(--color-learning-journey-card-shadow-color)',
        overflow: 'hidden',
        display: 'flex',
        flexDirection: 'column',
      }}
    >
      {/* Header - Fixed */}
      <div style={{
        background: 'linear-gradient(135deg, #ff6b35 0%, #ff8c42 100%)',
        padding: '24px',
        color: 'white',
        flexShrink: 0,
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '16px' }}>
          <div style={{
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: 'rgba(255, 255, 255, 0.3)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            fontSize: '24px',
            fontWeight: 'bold',
            border: '3px solid rgba(255, 255, 255, 0.5)',
          }}>
            {user.name?.charAt(0).toUpperCase() || 'U'}
          </div>
          <div style={{ flex: 1, minWidth: 0 }}>
            <h3 style={{
              fontSize: '18px',
              fontWeight: '600',
              margin: 0,
              marginBottom: '4px',
              whiteSpace: 'nowrap',
              overflow: 'hidden',
              textOverflow: 'ellipsis',
            }}>
              {user.name || 'User'}
            </h3>
            <p style={{
              fontSize: '13px',
              margin: 0,
              opacity: 0.95,
              whiteSpace: 'nowrap',
              overflow: 'hidden',
              textOverflow: 'ellipsis',
            }}>
              {user.email}
            </p>
          </div>
        </div>
      </div>

      {/* Scrollable Content */}
      <div style={{
        flex: 1,
        overflowY: 'auto',
        overflowX: 'hidden',
        padding: '20px',
      }}>
        {/* Experience Section */}
        <div style={{ marginBottom: '20px' }}>
          <h4 style={{
            fontSize: '11px',
            fontWeight: '600',
            textTransform: 'uppercase',
            letterSpacing: '0.5px',
            color: 'var(--color-muted)',
            marginBottom: '12px',
          }}>
            Experience Levels
          </h4>
          
           
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '10px'  }}>
            <InfoCard label="Software" value={capitalize(user.softwareBackground)} />
            <InfoCard label="Hardware" value={capitalize(user.hardwareBackground)} />
            <InfoCard label="Robotics" value={capitalize(user.roboticsExperience)} />
            <InfoCard label="AI/ML" value={capitalize(user.aiMlExperience)} />
          </div>
        </div>

        {/* Programming Languages */}
        {languages.length > 0 && (
          <div style={{ marginBottom: '20px' }}>
            <h4 style={{
              fontSize: '11px',
              fontWeight: '600',
              textTransform: 'uppercase',
              letterSpacing: '0.5px',
              color: 'var(--color-muted)',
              marginBottom: '12px',
            }}>
              Programming Languages
            </h4>
            <div style={{ display: 'flex', flexWrap: 'wrap', gap: '8px' }}>
              {languages.map((lang, index) => (
                <span
                  key={index}
                  style={{
                    padding: '6px 12px',
                    fontSize: '12px',
                    fontWeight: '500',
                    backgroundColor: 'var(--color-profile-language-bg-color)',
                    color: 'var(--color-hover-button-background)',
                    borderRadius: '20px',
                    border: '1px solid var(--color-learning-journey-card-shadow-color)',
                  }}
                >
                  {lang}
                </span>
              ))}
            </div>
          </div>
        )}

        {/* Special Features */}
        {(user.hasRosExperience || user.hasGpuAccess) && (
          <div style={{ marginBottom: '20px' }}>
            <h4 style={{
              fontSize: '11px',
              fontWeight: '600',
              textTransform: 'uppercase',
              letterSpacing: '0.5px',
              color: 'var(--color-muted)',
              marginBottom: '12px',
            }}>
              Access & Tools
            </h4>
            <div style={{ display: 'flex', flexWrap: 'wrap', gap: '8px' }}>
              {user.hasRosExperience && (
                <span style={{
                  padding: '6px 12px',
                  fontSize: '12px',
                  fontWeight: '500',
                  backgroundColor: 'var(--color-profile-language-bg-color)',
                  color: 'var(--color-hover-button-background)',
                  borderRadius: '20px',
                  border: '1px solid var(--color-learning-journey-card-shadow-color)',
                }}>
                  ROS Experience
                </span>
              )}
              {user.hasGpuAccess && (
                <span style={{
                  padding: '6px 12px',
                  fontSize: '12px',
                  fontWeight: '500',
                  backgroundColor: 'var(--color-badge-background)',
                  color: 'var(--color-accent)',
                  borderRadius: '20px',
                  border: '1px solid var(--color-learning-journey-card-shadow-color)',
                }}>
                  GPU Access
                </span>
              )}
            </div>
          </div>
        )}

        {/* Learning Goals */}
        {user.learningGoals && user.learningGoals.trim() !== '' && (
          <div>
            <h4 style={{
              fontSize: '11px',
              fontWeight: '600',
              textTransform: 'uppercase',
              letterSpacing: '0.5px',
              color: 'var(--color-muted)',
              marginBottom: '12px',
            }}>
              Learning Goals
            </h4>
            <div style={{
              backgroundColor: 'var(--color-physical-from-CTA-background)',
              padding: '14px',
              borderRadius: '8px',
              border: '1px solid var(--color-learning-journey-card-shadow-color)',
            }}>
              <p style={{
                fontSize: '13px',
                color: 'var(--color-description-text)',
                lineHeight: '1.6',
                margin: 0,
              }}>
                {user.learningGoals.length > 200
                  ? user.learningGoals.slice(0, 200) + "..."
                  : user.learningGoals}
              </p>
            </div>
          </div>
        )}
      </div>

      {/* Action Buttons - Fixed at Bottom */}
      <div style={{
        padding: '16px 20px',
        borderTop: '1px solid var(--color-learning-journey-card-shadow-color)',
        backgroundColor: 'var(--color-physical-from-CTA-background)',
        display: 'flex',
        flexDirection: 'column',
        gap: '10px',
        flexShrink: 0,
      }}>
        <button
          onClick={() => {
            onClose();
            window.location.href = '/profile';
          }}
          style={{
            width: '100%',
            padding: '12px',
            backgroundColor: 'var(--color-dark-button-background)',
            color: 'white',
            border: 'none',
            borderRadius: '8px',
            fontSize: '14px',
            fontWeight: '600',
            cursor: 'pointer',
            transition: 'all 0.2s',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            gap: '8px',
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.backgroundColor = 'var(--color-hover-button-background)';
            e.currentTarget.style.transform = 'translateY(-2px)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.backgroundColor = 'var(--color-dark-button-background)';
            e.currentTarget.style.transform = 'translateY(0)';
          }}
        >
          <svg width="18" height="18" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} 
                  d="M11 5H6a2 2 0 00-2 2v11a2 2 0 002 2h11a2 2 0 002-2v-5m-1.414-9.414a2 2 0 112.828 2.828L11.828 15H9v-2.828l8.586-8.586z" />
          </svg>
          <span>Edit Profile</span>
        </button>
        
        <button
  onClick={onLogout}
  style={{
    width: '100%',
    padding: '12px',
    backgroundColor: 'var(--color-bg)',
    color: '#dc2626',
    border: '2px solid #fecaca',
    borderRadius: '8px',
    fontSize: '14px',
    fontWeight: '600',
    cursor: 'pointer',
    transition: 'all 0.2s',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    gap: '8px',
  }}
  onMouseEnter={(e) => {
    e.currentTarget.style.backgroundColor = 'var(--color-physical-from-CTA-background)';
    e.currentTarget.style.borderColor = '#fca5a5';
  }}
  onMouseLeave={(e) => {
    e.currentTarget.style.backgroundColor = 'var(--color-bg)';
    e.currentTarget.style.borderColor = '#fecaca';
  }}
>
  <svg width="18" height="18" fill="none" stroke="currentColor" viewBox="0 0 24 24">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} 
          d="M17 16l4-4m0 0l-4-4m4 4H7m6 4v1a3 3 0 01-3 3H6a3 3 0 01-3-3V7a3 3 0 013-3h4a3 3 0 013 3v1" />
  </svg>
  <span>Logout</span>
</button>
      </div>
    </div>
  );
};

// Simple Info Card Component with Dark Mode
const InfoCard = ({ label, value }: { label: string; value: string }) => (
  <div style={{
    backgroundColor: 'var(--color-physical-from-CTA-background)',
    padding: '14px',
    borderRadius: '8px',
    border: '1px solid var(--color-profile-card-border-color)',
    textAlign: 'center',
  }}>
    <div style={{
      fontSize: '11px',
      fontWeight: '500',
      color: 'var(--color-muted)',
      marginBottom: '6px',
    }}>
      {label}
    </div>
    <div style={{
      fontSize: '13px',
      fontWeight: '600',
      color: 'var(--color-accent)',
    }}>
      {value}
    </div>
  </div>
);

const ProfileDropdown = memo(ProfileDropdownComponent);
export default ProfileDropdown;