from fastapi import Depends, HTTPException, status, Request
from typing import Optional
from schemas import UserSession
import jwt
from datetime import datetime, timedelta
import os
from pydantic import BaseModel


class SessionValidator:
    """
    Enhanced session validation utility for validating user sessions
    """

    def __init__(self):
        # In a real implementation, this would come from environment variables
        self.secret = os.getenv("JWT_SECRET", "your-secret-key-default")
        self.algorithm = "HS256"

    async def validate_session(self, token: str) -> Optional[UserSession]:
        """
        Validate user session from JWT token
        Returns UserSession if valid, None if invalid/expired
        """
        try:
            # Decode the token
            payload = jwt.decode(token, self.secret, algorithms=[self.algorithm])

            user_id = payload.get("user_id")
            exp = payload.get("exp")

            if user_id is None:
                return None

            # Check if token is expired
            if exp and datetime.fromtimestamp(exp) < datetime.utcnow():
                return None

            # Create and return a user session
            return UserSession(
                user_id=user_id,
                authenticated=True,
                session_token=token
            )
        except jwt.ExpiredSignatureError:
            # Token has expired
            return None
        except jwt.InvalidTokenError:
            # Invalid token
            return None
        except Exception as e:
            # Other error occurred
            print(f"Session validation error: {str(e)}")  # In production, use proper logging
            return None

    async def validate_user_session(self, user_id: str, session_token: str) -> bool:
        """
        Validate if a specific user session is still valid
        """
        user_session = await self.validate_session(session_token)
        return user_session is not None and user_session.user_id == user_id

    def create_token(self, user_id: str) -> str:
        """
        Create a JWT token for a user
        """
        payload = {
            "user_id": user_id,
            "exp": datetime.utcnow() + timedelta(hours=24),  # Token expires in 24 hours
            "iat": datetime.utcnow()
        }
        return jwt.encode(payload, self.secret, algorithm=self.algorithm)

    def get_user_id_from_token(self, token: str) -> Optional[str]:
        """
        Extract user_id from token without full validation
        Used for logging and tracking purposes
        """
        try:
            payload = jwt.decode(token, self.secret, algorithms=[self.algorithm], options={"verify_exp": False})
            return payload.get("user_id")
        except:
            return None


# Global instance
session_validator = SessionValidator()


# Dependency for getting current user from token
async def get_current_user(request: Request) -> UserSession:
    """
    Dependency to get current user from authorization header
    """
    auth_header = request.headers.get("Authorization")

    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing or invalid format",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = auth_header[7:]  # Remove "Bearer " prefix

    user_session = await session_validator.validate_session(token)

    if not user_session or not user_session.authenticated:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user_session


# Enhanced dependency that checks session validity mid-request
async def get_current_user_with_session_check(request: Request) -> UserSession:
    """
    Dependency to get current user and verify session is still valid during the request
    Handles session expiration that might occur during long-running operations
    """
    auth_header = request.headers.get("Authorization")

    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing or invalid format",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = auth_header[7:]  # Remove "Bearer " prefix

    user_session = await session_validator.validate_session(token)

    if not user_session or not user_session.authenticated:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Additional check: verify the session is still valid at the start of processing
    # This can be used to check if the session has been revoked or invalidated
    user_id = session_validator.get_user_id_from_token(token)
    if not user_id or not await session_validator.validate_user_session(user_id, token):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session expired or invalidated during request",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user_session


# Alternative dependency for optional authentication (for testing)
async def get_current_user_optional(request: Request) -> Optional[UserSession]:
    """
    Dependency to get current user from authorization header (optional)
    Returns None if no valid token is provided
    """
    auth_header = request.headers.get("Authorization")

    if not auth_header or not auth_header.startswith("Bearer "):
        return None

    token = auth_header[7:]  # Remove "Bearer " prefix

    user_session = await session_validator.validate_session(token)

    if not user_session or not user_session.authenticated:
        return None

    return user_session


# Utility function for checking if user is authenticated without raising exception
async def is_user_authenticated(request: Request) -> bool:
    """
    Check if user is authenticated without raising HTTP exceptions
    Returns True if authenticated, False otherwise
    """
    auth_header = request.headers.get("Authorization")

    if not auth_header or not auth_header.startswith("Bearer "):
        return False

    token = auth_header[7:]  # Remove "Bearer " prefix

    user_session = await session_validator.validate_session(token)

    return user_session is not None and user_session.authenticated