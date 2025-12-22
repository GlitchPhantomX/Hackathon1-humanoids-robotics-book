import React, { useState, useEffect } from 'react';
import personalizationService from '../services/personalization';

interface PersonalizeButtonProps {
  chapterId?: string;
  chapterContent?: string;
}

interface PersonalizationState {
  isPersonalized: boolean;
  isLoading: boolean;
  hasError: boolean;
  originalContent: string;
  personalizedContent: string;
}

const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({ chapterId, chapterContent }) => {
  const [state, setState] = useState<PersonalizationState>({
    isPersonalized: false,
    isLoading: false,
    hasError: false,
    originalContent: chapterContent || '',
    personalizedContent: chapterContent || ''
  });

  const [loadingProgress, setLoadingProgress] = useState<number>(0);
  const loadingStartTime = React.useRef<number | null>(null);

  useEffect(() => {
    if (!chapterContent) {
      const articleElement = document.querySelector('article');
      if (articleElement) {
        const fullHTML = articleElement.innerHTML;
        setState(prev => ({
          ...prev,
          originalContent: fullHTML,
          personalizedContent: fullHTML
        }));
      }
    }
  }, [chapterContent]);

  const fetchUserProfile = async (): Promise<any> => {
    try {
      return {
        softwareBackground: 'beginner',
        hardwareBackground: 'maker',
        programmingLanguages: 'Python',
        roboticsExperience: 'none',
        aiMlExperience: 'beginner',
        hasRosExperience: false,
        hasGpuAccess: true,
        learningGoals: 'Learn robotics fundamentals'
      };
    } catch (error) {
      console.error('Error fetching user profile:', error);
      return null;
    }
  };

  const handlePersonalize = async () => {
    if (state.isPersonalized || state.isLoading) return;

    loadingStartTime.current = Date.now();
    setLoadingProgress(0);
    setState(prev => ({ ...prev, isLoading: true, hasError: false }));

    const progressInterval = setInterval(() => {
      setLoadingProgress(prev => {
        if (prev >= 90) return prev;
        return prev + 3;
      });
    }, 500);

    try {
      console.log('🎯 Starting personalization...');
      
      const userProfile = await fetchUserProfile();
      if (!userProfile) {
        throw new Error('Could not fetch user profile');
      }

      let contentToPersonalize = state.originalContent;
      
      if (!contentToPersonalize) {
        const articleElement = document.querySelector('article');
        if (articleElement) {
          const textContent = articleElement.textContent || '';
          contentToPersonalize = textContent;
        }
      }

      console.log(`📄 Content length: ${contentToPersonalize.length} chars`);

      const timeoutPromise = new Promise<never>((_, reject) => {
        setTimeout(() => {
          console.error('❌ Timeout after 60 seconds');
          reject(new Error('Personalization request timed out after 60 seconds'));
        }, 60000);
      });

      console.log('📡 Calling personalization API...');
      const responsePromise = personalizationService.personalizeChapter({
        chapterId: chapterId || window.location.pathname,
        chapterContent: contentToPersonalize,
        userProfile: userProfile
      });

      const response = await Promise.race([responsePromise, timeoutPromise]);

      console.log('✅ Personalization complete!');
      console.log('📊 Response length:', response.personalizedContent.length);
      
      // ✅ Clear interval and set to 100% IMMEDIATELY
      clearInterval(progressInterval);
      setLoadingProgress(100);

      // Store personalized content
      sessionStorage.setItem('personalizedContent', response.personalizedContent);
      sessionStorage.setItem('personalizedChapter', chapterId || window.location.pathname);
      sessionStorage.setItem('isPersonalized', 'true');

      // ✅ Update state WITHOUT alert (alert was blocking progress)
      setState({
        isPersonalized: true,
        isLoading: false,
        hasError: false,
        originalContent: state.originalContent,
        personalizedContent: response.personalizedContent
      });

      // ✅ Reset progress smoothly
      setTimeout(() => setLoadingProgress(0), 500);

    } catch (error: any) {
      console.error('❌ Personalization failed:', error);

      clearInterval(progressInterval);
      setLoadingProgress(0);

      // ✅ Keep alert for errors only
      alert(
        `Personalization failed!\n\n` +
        `Error: ${error.message}\n\n` +
        `Troubleshooting:\n` +
        `1. Check if backend is running at http://localhost:8002\n` +
        `2. Open browser console (F12) for detailed logs\n` +
        `3. Visit http://localhost:8002/docs to test the API\n` +
        `4. Check if OPENAI_API_KEY is set in backend .env file`
      );

      setState(prev => ({
        ...prev,
        isLoading: false,
        hasError: true
      }));
    }
  };

  const handleReset = () => {
    sessionStorage.removeItem('personalizedContent');
    sessionStorage.removeItem('personalizedChapter');
    sessionStorage.removeItem('isPersonalized');
    window.location.reload();
  };

  useEffect(() => {
    const isPersonalized = sessionStorage.getItem('isPersonalized') === 'true';
    const personalizedChapter = sessionStorage.getItem('personalizedChapter');
    const currentChapter = chapterId || window.location.pathname;

    if (isPersonalized && personalizedChapter === currentChapter) {
      setState(prev => ({ ...prev, isPersonalized: true }));
    }
  }, [chapterId]);

  if (state.isLoading) {
    return (
      <div style={{
        padding: '16px 24px',
        backgroundColor: '#fff3e0',
        border: '2px solid #FFA500',
        borderRadius: '8px',
        marginBottom: '20px',
        boxShadow: '0 2px 8px rgba(255,165,0,0.2)'
      }}>
        <div style={{
          display: 'flex',
          alignItems: 'center',
          gap: '16px',
          marginBottom: '12px'
        }}>
          <div style={{
            width: '28px',
            height: '28px',
            border: '3px solid #FFA500',
            borderTop: '3px solid transparent',
            borderRadius: '50%',
            animation: 'spin 1s linear infinite'
          }}></div>
          <div style={{ flex: 1 }}>
            <div style={{ 
              color: '#e65100', 
              fontWeight: 'bold', 
              fontSize: '16px',
              marginBottom: '4px'
            }}>
              ✨ Personalizing content for you...
            </div>
            <div style={{ color: '#666', fontSize: '13px' }}>
              This may take 20-40 seconds. Please wait...
            </div>
          </div>
        </div>
        
        <div style={{
          width: '100%',
          height: '10px',
          backgroundColor: '#ffe0b2',
          borderRadius: '5px',
          overflow: 'hidden',
          boxShadow: 'inset 0 1px 3px rgba(0,0,0,0.1)'
        }}>
          <div style={{
            width: `${loadingProgress}%`,
            height: '100%',
            background: 'linear-gradient(90deg, #FFA500, #FF8C00)',
            transition: 'width 0.3s ease',
            borderRadius: '5px',
            boxShadow: '0 0 10px rgba(255,165,0,0.5)'
          }}></div>
        </div>
        
        <div style={{
          marginTop: '8px',
          fontSize: '12px',
          color: '#999',
          textAlign: 'right'
        }}>
          {loadingProgress}% complete
        </div>
      </div>
    );
  }

  return (
    <div style={{ marginBottom: '20px' }}>
      <button
        onClick={state.isPersonalized ? handleReset : handlePersonalize}
        disabled={state.isLoading}
        style={{
          backgroundColor: state.isPersonalized ? '#4CAF50' : '#FFA500',
          color: 'white',
          border: 'none',
          padding: '12px 24px',
          borderRadius: '8px',
          cursor: state.isLoading ? 'not-allowed' : 'pointer',
          fontSize: '15px',
          fontWeight: '600',
          marginRight: '12px',
          opacity: state.isLoading ? 0.6 : 1,
          minWidth: '220px',
          minHeight: '44px',
          boxShadow: '0 2px 6px rgba(0,0,0,0.15)',
          transition: 'all 0.2s ease',
          display: 'inline-flex',
          alignItems: 'center',
          justifyContent: 'center',
          gap: '8px'
        }}
        onMouseEnter={(e) => {
          if (!state.isLoading) {
            e.currentTarget.style.transform = 'translateY(-2px)';
            e.currentTarget.style.boxShadow = '0 4px 12px rgba(0,0,0,0.2)';
          }
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'translateY(0)';
          e.currentTarget.style.boxShadow = '0 2px 6px rgba(0,0,0,0.15)';
        }}
      >
        {state.isPersonalized ? (
          <>
            <span>✓</span>
            <span>Personalized View</span>
          </>
        ) : (
          <>
            <span>🎯</span>
            <span>Personalize for Me</span>
          </>
        )}
      </button>

      {state.isPersonalized && (
        <span style={{
          fontSize: '13px',
          color: '#4CAF50',
          fontWeight: '600',
          fontStyle: 'italic'
        }}>
          ✨ Adapted to beginner level • <a 
            onClick={handleReset} 
            style={{ 
              cursor: 'pointer', 
              textDecoration: 'underline',
              color: '#4CAF50'
            }}
          >
            Restore original
          </a>
        </span>
      )}

      {state.hasError && (
        <div style={{
          marginTop: '12px',
          padding: '12px 16px',
          backgroundColor: '#ffebee',
          border: '1px solid #ef5350',
          borderRadius: '6px',
          fontSize: '13px',
          color: '#c62828'
        }}>
          <strong>⚠️ Personalization Failed</strong>
          <div style={{ marginTop: '4px' }}>
            Check browser console (F12) for details. Backend might not be running.
          </div>
        </div>
      )}

      <style>{`
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(5px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
    </div>
  );
};

export default PersonalizeButton;