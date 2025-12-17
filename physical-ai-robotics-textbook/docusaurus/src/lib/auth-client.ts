import { createAuthClient } from "better-auth/react";

// Docusaurus uses window.location.origin or custom fields in docusaurus.config.js
// For client-side code, you can use window.location.origin or define a custom field
const getApiUrl = () => {
  if (typeof window === 'undefined') {
    return "http://localhost:5000";
  }
  
  // Option 1: Use a different domain in production
  // return window.location.hostname === 'localhost' 
  //   ? 'http://localhost:5000' 
  //   : 'https://your-production-api.com';
  
  // Option 2: Use same domain with /api path
  // return `${window.location.origin}/api`;
  
  // Option 3: For development, just use localhost
  return "http://localhost:5000";
};

const API_URL = getApiUrl();

// Real Better Auth client configuration
export const authClient = createAuthClient({
  baseURL: API_URL,
  fetchOptions: {
    credentials: 'include', // Important for cookies
  },
});

export const { signIn, signUp, signOut, useSession } = authClient;

// Keep accessibility functions
let announcementElement: HTMLDivElement | null = null;

export const announceToScreenReader = (message: string) => {
  if (typeof window === 'undefined') return;
  
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
  announcementElement.textContent = message;
};

export const focusElement = (elementId: string) => {
  if (typeof window === 'undefined') return;
  
  const element = document.getElementById(elementId);
  if (element) {
    element.focus();
    if (!element.hasAttribute('tabindex')) {
      element.setAttribute('tabindex', '-1');
      element.focus();
    }
  }
};