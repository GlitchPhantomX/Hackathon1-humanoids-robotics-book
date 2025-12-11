import React from 'react';
import { AuthProvider } from 'better-auth/react';
import { authClient } from '../../lib/auth-client';

interface AuthProviderWrapperProps {
  children: React.ReactNode;
}

const AuthProviderWrapper: React.FC<AuthProviderWrapperProps> = ({ children }) => {
  return (
    <AuthProvider client={authClient}>
      {children}
    </AuthProvider>
  );
};

export default AuthProviderWrapper;