import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { authClient } from '../lib/auth-client';
import type { User } from '../components/auth/types';

interface ProfileFormData {
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

export default function ProfilePage() {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error', text: string } | null>(null);

  const [formData, setFormData] = useState<ProfileFormData>({
    name: '',
    email: '',
    softwareBackground: '',
    hardwareBackground: '',
    programmingLanguages: '',
    roboticsExperience: '',
    aiMlExperience: '',
    hasRosExperience: false,
    hasGpuAccess: false,
    learningGoals: '',
  });

  useEffect(() => {
    fetchUserProfile();
  }, []);

  const fetchUserProfile = async () => {
    try {
      const session = await authClient.getSession();
      if (session?.data?.user) {
        const userData = session.data.user as User;
        setUser(userData);
        setFormData({
          name: userData.name || '',
          email: userData.email || '',
          softwareBackground: userData.softwareBackground || '',
          hardwareBackground: userData.hardwareBackground || '',
          programmingLanguages: userData.programmingLanguages || '',
          roboticsExperience: userData.roboticsExperience || '',
          aiMlExperience: userData.aiMlExperience || '',
          hasRosExperience: userData.hasRosExperience || false,
          hasGpuAccess: userData.hasGpuAccess || false,
          learningGoals: userData.learningGoals || '',
        });
      } else {
        window.location.href = '/';
      }
    } catch (error) {
      console.error('Failed to fetch profile:', error);
      window.location.href = '/';
    } finally {
      setLoading(false);
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value, type } = e.target;
    const checked = (e.target as HTMLInputElement).checked;

    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
  e.preventDefault();
  setSaving(true);
  setMessage(null);

  try {
    // ✅ DYNAMIC API URL
    const API_URL = window.location.hostname === 'localhost' 
      ? 'http://localhost:5000' 
      : 'https://hackathon1-humanoids-robotics-book-production.up.railway.app';

    const response = await fetch(`${API_URL}/api/user/profile`, {
      method: 'PATCH',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify(formData),
    });

    if (response.ok) {
      setMessage({ type: 'success', text: 'Profile updated successfully! Redirecting...' });
      
      window.dispatchEvent(new CustomEvent('profileUpdated'));
      localStorage.setItem('profileLastUpdated', Date.now().toString());
      
      setTimeout(() => {
        window.location.href = '/';
      }, 1500);
    } else {
      throw new Error('Failed to update profile');
    }
  } catch (error) {
    console.error('Update error:', error);
    setMessage({ type: 'error', text: 'Failed to update profile. Please try again.' });
    setSaving(false);
  }
};

  if (loading) {
    return (
      <Layout title="Profile">
        <div style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          minHeight: '60vh',
        }}>
          <div style={{
            width: '50px',
            height: '50px',
            border: '4px solid #f3f4f6',
            borderTop: '4px solid #ff6b35',
            borderRadius: '50%',
            animation: 'spin 1s linear infinite',
          }}></div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Edit Profile">
      <style>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
      
      <div style={{
        maxWidth: '900px',
        margin: '0 auto',
        padding: '40px 20px',
      }}>
        <div style={{
          backgroundColor: 'white',
          borderRadius: '16px',
          boxShadow: '0 4px 20px rgba(0, 0, 0, 0.08)',
          padding: '40px',
        }}>
          {/* Header */}
          <div style={{ marginBottom: '40px' }}>
            <h1 style={{
              fontSize: '32px',
              fontWeight: '700',
              color: '#1f2937',
              marginBottom: '8px',
            }}>
              Edit Profile
            </h1>
            <p style={{
              fontSize: '16px',
              color: '#6b7280',
            }}>
              Update your personal information and preferences
            </p>
          </div>

          {/* Success/Error Message */}
          {message && (
            <div style={{
              padding: '16px',
              borderRadius: '8px',
              marginBottom: '30px',
              backgroundColor: message.type === 'success' ? '#f0fdf4' : '#fef2f2',
              border: `1px solid ${message.type === 'success' ? '#86efac' : '#fecaca'}`,
              color: message.type === 'success' ? '#16a34a' : '#dc2626',
              display: 'flex',
              alignItems: 'center',
              gap: '10px',
            }}>
              <span style={{ fontSize: '18px' }}>
                {message.type === 'success' ? '✓' : '✕'}
              </span>
              <span style={{ fontWeight: '500' }}>{message.text}</span>
            </div>
          )}

          <form onSubmit={handleSubmit}>
            {/* Basic Info */}
            <div style={{
              display: 'grid',
              gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))',
              gap: '24px',
              marginBottom: '30px',
            }}>
              <div>
                <label style={{
                  display: 'block',
                  fontSize: '14px',
                  fontWeight: '500',
                  color: '#374151',
                  marginBottom: '8px',
                }}>
                  Full Name
                </label>
                <input
                  type="text"
                  name="name"
                  value={formData.name}
                  onChange={handleChange}
                  required
                  style={{
                    width: '100%',
                    padding: '12px',
                    border: '1px solid #d1d5db',
                    borderRadius: '8px',
                    fontSize: '14px',
                    transition: 'all 0.2s',
                  }}
                  onFocus={(e) => {
                    e.target.style.borderColor = '#ff6b35';
                    e.target.style.outline = 'none';
                    e.target.style.boxShadow = '0 0 0 3px rgba(255, 107, 53, 0.1)';
                  }}
                  onBlur={(e) => {
                    e.target.style.borderColor = '#d1d5db';
                    e.target.style.boxShadow = 'none';
                  }}
                />
              </div>

              <div>
                <label style={{
                  display: 'block',
                  fontSize: '14px',
                  fontWeight: '500',
                  color: '#374151',
                  marginBottom: '8px',
                }}>
                  Email
                </label>
                <input
                  type="email"
                  name="email"
                  value={formData.email}
                  disabled
                  style={{
                    width: '100%',
                    padding: '12px',
                    border: '1px solid #d1d5db',
                    borderRadius: '8px',
                    fontSize: '14px',
                    backgroundColor: '#f9fafb',
                    color: '#9ca3af',
                    cursor: 'not-allowed',
                  }}
                />
              </div>
            </div>

            {/* Experience Levels */}
            <div style={{
              display: 'grid',
              gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))',
              gap: '24px',
              marginBottom: '30px',
            }}>
              {[
                { name: 'softwareBackground', label: 'Software Background', options: ['beginner', 'intermediate', 'advanced', 'expert'] },
                { name: 'hardwareBackground', label: 'Hardware Background', options: ['none', 'basic', 'intermediate', 'advanced'] },
                { name: 'roboticsExperience', label: 'Robotics Experience', options: ['none', 'hobbyist', 'academic', 'professional'] },
                { name: 'aiMlExperience', label: 'AI/ML Experience', options: ['none', 'basic', 'intermediate', 'advanced'] },
              ].map((field) => (
                <div key={field.name}>
                  <label style={{
                    display: 'block',
                    fontSize: '14px',
                    fontWeight: '500',
                    color: '#374151',
                    marginBottom: '8px',
                  }}>
                    {field.label}
                  </label>
                  <select
                    name={field.name}
                    value={formData[field.name as keyof ProfileFormData] as string}
                    onChange={handleChange}
                    style={{
                      width: '100%',
                      padding: '12px',
                      border: '1px solid #d1d5db',
                      borderRadius: '8px',
                      fontSize: '14px',
                      transition: 'all 0.2s',
                      cursor: 'pointer',
                    }}
                    onFocus={(e) => {
                      e.target.style.borderColor = '#ff6b35';
                      e.target.style.outline = 'none';
                      e.target.style.boxShadow = '0 0 0 3px rgba(255, 107, 53, 0.1)';
                    }}
                    onBlur={(e) => {
                      e.target.style.borderColor = '#d1d5db';
                      e.target.style.boxShadow = 'none';
                    }}
                  >
                    <option value="">Select level</option>
                    {field.options.map(option => (
                      <option key={option} value={option}>
                        {option.charAt(0).toUpperCase() + option.slice(1)}
                      </option>
                    ))}
                  </select>
                </div>
              ))}
            </div>

            {/* Programming Languages */}
            <div style={{ marginBottom: '30px' }}>
              <label style={{
                display: 'block',
                fontSize: '14px',
                fontWeight: '500',
                color: '#374151',
                marginBottom: '8px',
              }}>
                Programming Languages (comma-separated)
              </label>
              <input
                type="text"
                name="programmingLanguages"
                value={formData.programmingLanguages}
                onChange={handleChange}
                placeholder="e.g., Python, JavaScript, C++"
                style={{
                  width: '100%',
                  padding: '12px',
                  border: '1px solid #d1d5db',
                  borderRadius: '8px',
                  fontSize: '14px',
                  transition: 'all 0.2s',
                }}
                onFocus={(e) => {
                  e.target.style.borderColor = '#ff6b35';
                  e.target.style.outline = 'none';
                  e.target.style.boxShadow = '0 0 0 3px rgba(255, 107, 53, 0.1)';
                }}
                onBlur={(e) => {
                  e.target.style.borderColor = '#d1d5db';
                  e.target.style.boxShadow = 'none';
                }}
              />
            </div>

            {/* Checkboxes */}
            <div style={{
              display: 'flex',
              gap: '30px',
              marginBottom: '30px',
              flexWrap: 'wrap',
            }}>
              {[
                { name: 'hasRosExperience', label: 'I have ROS experience' },
                { name: 'hasGpuAccess', label: 'I have GPU access' },
              ].map((field) => (
                <label key={field.name} style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '10px',
                  cursor: 'pointer',
                  fontSize: '14px',
                  color: '#374151',
                  fontWeight: '500',
                }}>
                  <input
                    type="checkbox"
                    name={field.name}
                    checked={formData[field.name as keyof ProfileFormData] as boolean}
                    onChange={handleChange}
                    style={{
                      width: '18px',
                      height: '18px',
                      cursor: 'pointer',
                      accentColor: '#ff6b35',
                    }}
                  />
                  <span>{field.label}</span>
                </label>
              ))}
            </div>

            {/* Learning Goals */}
            <div style={{ marginBottom: '30px' }}>
              <label style={{
                display: 'block',
                fontSize: '14px',
                fontWeight: '500',
                color: '#374151',
                marginBottom: '8px',
              }}>
                Learning Goals
              </label>
              <textarea
                name="learningGoals"
                value={formData.learningGoals}
                onChange={handleChange}
                rows={4}
                placeholder="What do you want to learn?"
                style={{
                  width: '100%',
                  padding: '12px',
                  border: '1px solid #d1d5db',
                  borderRadius: '8px',
                  fontSize: '14px',
                  transition: 'all 0.2s',
                  resize: 'vertical',
                }}
                onFocus={(e) => {
                  e.target.style.borderColor = '#ff6b35';
                  e.target.style.outline = 'none';
                  e.target.style.boxShadow = '0 0 0 3px rgba(255, 107, 53, 0.1)';
                }}
                onBlur={(e) => {
                  e.target.style.borderColor = '#d1d5db';
                  e.target.style.boxShadow = 'none';
                }}
              />
            </div>

            {/* Buttons */}
            <div style={{ display: 'flex', gap: '16px' }}>
              <button
                type="submit"
                disabled={saving}
                style={{
                  flex: 1,
                  padding: '14px',
                  backgroundColor: saving ? '#fed7aa' : '#ff6b35',
                  color: 'white',
                  border: 'none',
                  borderRadius: '8px',
                  fontSize: '16px',
                  fontWeight: '600',
                  cursor: saving ? 'not-allowed' : 'pointer',
                  transition: 'all 0.2s',
                }}
                onMouseEnter={(e) => {
                  if (!saving) e.currentTarget.style.backgroundColor = '#ff5722';
                }}
                onMouseLeave={(e) => {
                  if (!saving) e.currentTarget.style.backgroundColor = '#ff6b35';
                }}
              >
                {saving ? 'Saving...' : 'Save Changes'}
              </button>
              
              <button
                type="button"
                onClick={() => window.location.href = '/'}
                disabled={saving}
                style={{
                  padding: '14px 28px',
                  backgroundColor: 'white',
                  color: '#6b7280',
                  border: '2px solid #d1d5db',
                  borderRadius: '8px',
                  fontSize: '16px',
                  fontWeight: '600',
                  cursor: saving ? 'not-allowed' : 'pointer',
                  transition: 'all 0.2s',
                }}
                onMouseEnter={(e) => {
                  if (!saving) {
                    e.currentTarget.style.backgroundColor = '#f9fafb';
                    e.currentTarget.style.borderColor = '#9ca3af';
                  }
                }}
                onMouseLeave={(e) => {
                  if (!saving) {
                    e.currentTarget.style.backgroundColor = 'white';
                    e.currentTarget.style.borderColor = '#d1d5db';
                  }
                }}
              >
                Cancel
              </button>
            </div>
          </form>
        </div>
      </div>
    </Layout>
  );
}