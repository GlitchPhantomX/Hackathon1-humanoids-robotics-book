import { createAuthClient } from 'better-auth/client';

// Create the auth client for the frontend
export const authClient = createAuthClient({
  baseURL: process.env.NODE_ENV === 'production'
    ? (process.env.NEXT_PUBLIC_BETTER_AUTH_URL || 'https://your-domain.com')
    : 'http://localhost:5000',
  fetchOptions: {
    credentials: 'include', // Important for cookie handling
  },
});

// Export individual functions for easier use
export const { signIn, signOut } = authClient;

// Accessibility-focused helper functions
// Create a single announcement element to reuse
let announcementElement: HTMLDivElement | null = null;

export const announceToScreenReader = (message: string) => {
  // Create a single reusable element to announce messages to screen readers
  if (!announcementElement) {
    announcementElement = document.createElement('div');
    announcementElement.setAttribute('aria-live', 'polite');
    announcementElement.setAttribute('aria-atomic', 'true');
    announcementElement.className = 'visually-hidden';
    announcementElement.style.position = 'absolute';
    announcementElement.style.width = '1px';
    announcementElement.style.height = '1px';
    announcementElement.style.padding = '0';
    announcementElement.style.margin = '-1px';
    announcementElement.style.overflow = 'hidden';
    announcementElement.style.clip = 'rect(0, 0, 0, 0)';
    announcementElement.style.whiteSpace = 'nowrap';
    announcementElement.style.border = '0';
    document.body.appendChild(announcementElement);
  }

  // Update the content to trigger the screen reader announcement
  announcementElement.textContent = message;
};

// Function to focus on specific elements with accessibility in mind
export const focusElement = (elementId: string) => {
  const element = document.getElementById(elementId);
  if (element) {
    element.focus();
    // If the element doesn't support focus, try to make it focusable
    if (!element.hasAttribute('tabindex')) {
      element.setAttribute('tabindex', '-1');
      element.focus();
    }
  }
};