// Personalization API service

interface PersonalizeChapterRequest {
  chapterId: string;
  chapterContent: string;
  userProfile: {
    softwareBackground: string;
    hardwareBackground: string;
    programmingLanguages: string;
    roboticsExperience: string;
    aiMlExperience: string;
    hasRosExperience: boolean;
    hasGpuAccess: boolean;
    learningGoals: string;
  };
}

interface PersonalizeChapterResponse {
  personalizedContent: string;
  processingTimeMs: number;
  sectionId?: string;
}

interface ErrorResponse {
  error: string;
  fallback_content?: string;
  code: string;
}

class PersonalizationService {
  private baseUrl: string;

  constructor() {
    // ✅ FIX: Use hardcoded URL or check if window exists
    this.baseUrl = 'http://localhost:8002'; // ✅ Direct URL instead of process.env
  }

  async personalizeChapter(request: PersonalizeChapterRequest): Promise<PersonalizeChapterResponse> {
    try {
      // T016: Create personalization API client with proper error handling
      const response = await fetch(`${this.baseUrl}/api/personalize/chapter`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Include authentication token if available
          ...this.getAuthHeaders()
        },
        body: JSON.stringify(request)
      });

      if (!response.ok) {
        // Handle error response
        const errorData: ErrorResponse = await response.json().catch(() => ({} as ErrorResponse));

        // T021: Implement fallback to original content on failure
        if (errorData.fallback_content) {
          return {
            personalizedContent: errorData.fallback_content,
            processingTimeMs: 0
          };
        }

        throw new Error(errorData.error || `HTTP error! status: ${response.status}`);
      }

      const result: PersonalizeChapterResponse = await response.json();
      return result;
    } catch (error) {
      console.error('Personalization API call failed:', error);

      // T021: Fallback to original content on failure
      return {
        personalizedContent: request.chapterContent,
        processingTimeMs: 0
      };
    }
  }

  private getAuthHeaders(): Record<string, string> {
    // In a real implementation, this would extract the auth token
    // from the user's session/cookies
    const token = this.getAuthToken();
    if (token) {
      return {
        'Authorization': `Bearer ${token}`
      };
    }
    return {};
  }

  private getAuthToken(): string | null {
    // ✅ FIX: Check if localStorage exists (browser environment)
    if (typeof window !== 'undefined' && window.localStorage) {
      return localStorage.getItem('auth_token');
    }
    return null;
  }
}

export default new PersonalizationService();