import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from typing import Optional
from models.personalization import ErrorResponse
import logging


class PersonalizationErrorHandler:
    """
    Standardized error handling for personalization service
    """

    @staticmethod
    def create_error_response(error_msg: str,
                            fallback_content: Optional[str] = None,
                            error_code: str = "PERSONALIZATION_ERROR") -> ErrorResponse:
        """
        Create a standardized error response

        Args:
            error_msg: The error message to include
            fallback_content: Original content to return as fallback
            error_code: Standardized error code

        Returns:
            ErrorResponse model instance
        """
        return ErrorResponse(
            error=error_msg,
            fallback_content=fallback_content,
            code=error_code
        )

    @staticmethod
    def handle_personalization_error(exception: Exception,
                                   original_content: str,
                                   logger: logging.Logger) -> ErrorResponse:
        """
        Handle personalization errors and return appropriate fallback

        Args:
            exception: The exception that occurred
            original_content: Original content to use as fallback
            logger: Logger instance for error logging

        Returns:
            ErrorResponse model instance
        """
        error_msg = str(exception)
        logger.error(f"Personalization error: {error_msg}", exc_info=True)

        # Determine error code based on exception type
        if isinstance(exception, TimeoutError):
            error_code = "AI_SERVICE_TIMEOUT"
        elif "API" in error_msg.upper() or "KEY" in error_msg.upper():
            error_code = "AI_SERVICE_AUTH_ERROR"
        elif "quota" in error_msg.lower() or "credit" in error_msg.lower():
            error_code = "AI_SERVICE_QUOTA_EXCEEDED"
        else:
            error_code = "PERSONALIZATION_ERROR"

        return PersonalizationErrorHandler.create_error_response(
            error_msg=f"Personalization failed: {error_msg}",
            fallback_content=original_content,
            error_code=error_code
        )

    @staticmethod
    def handle_api_error(status_code: int,
                        message: str,
                        fallback_content: Optional[str] = None) -> ErrorResponse:
        """
        Handle API-level errors and return appropriate response

        Args:
            status_code: HTTP status code
            message: Error message
            fallback_content: Original content to return as fallback

        Returns:
            ErrorResponse model instance
        """
        # Map status codes to error codes
        if status_code == 400:
            error_code = "INVALID_REQUEST"
        elif status_code == 401:
            error_code = "UNAUTHORIZED"
        elif status_code == 403:
            error_code = "FORBIDDEN"
        elif status_code == 429:
            error_code = "RATE_LIMITED"
        elif status_code >= 500:
            error_code = "SERVER_ERROR"
        else:
            error_code = "UNKNOWN_ERROR"

        return PersonalizationErrorHandler.create_error_response(
            error_msg=message,
            fallback_content=fallback_content,
            error_code=error_code
        )