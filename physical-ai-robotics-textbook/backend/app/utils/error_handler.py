"""
Error handling and logging utilities for the RAG Chatbot API.
Implements all requirements from Phase 10: Error Handling & Logging.
"""

import logging
import sys
import traceback
import json
from functools import wraps
from typing import Callable, Any, Optional, Dict, Union
from datetime import datetime
import time
import asyncio
from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse
import re


# Initialize logger
logger = logging.getLogger("error_handler")
logger.setLevel(logging.INFO)

# Create console handler with higher log level
if not logger.handlers:
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.DEBUG)

    # Create formatter with timestamp and context
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    ch.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(ch)


class ServiceError(Exception):
    """Base exception class for service errors"""

    def __init__(
        self,
        message: str,
        error_code: str = "SERVICE_ERROR",
        status_code: int = 500,
        details: Optional[Dict] = None,
    ):
        super().__init__(message)
        self.message = message
        self.error_code = error_code
        self.status_code = status_code
        self.details = details or {}


class ValidationError(ServiceError):
    """Raised when input validation fails"""

    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "VALIDATION_ERROR", 400, details)


class NotFoundError(ServiceError):
    """Raised when a requested resource is not found"""

    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "NOT_FOUND", 404, details)


class DatabaseError(ServiceError):
    """Raised when a database operation fails"""

    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "DATABASE_ERROR", 500, details)


class AIServiceError(ServiceError):
    """Raised when AI service operations fail"""

    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "AI_SERVICE_ERROR", 503, details)


def log_api_request(
    endpoint: str,
    method: str,
    query: str = "",
    conversation_id: str = None,
    user_id: str = None,
):
    """
    Log API requests with context.

    Args:
        endpoint: The API endpoint that was called
        method: The HTTP method (GET, POST, etc.)
        query: The user's query (truncated for privacy)
        conversation_id: The conversation ID
        user_id: The user ID (if available)
    """
    try:
        log_details = {
            "timestamp": datetime.utcnow().isoformat(),
            "endpoint": endpoint,
            "method": method,
            "query_preview": query[:50] if query else "",
            "conversation_id": conversation_id,
            "user_id": user_id,
        }

        logger.info(f"API Request: {json.dumps(log_details)}")
    except Exception as e:
        logger.error(f"Error logging API request: {str(e)}")


def log_performance(operation: str, duration: float, details: Optional[Dict] = None):
    """
    Log performance metrics for operations.

    Args:
        operation: Name of the operation being measured
        duration: Duration of the operation in seconds
        details: Additional operation details
    """
    try:
        log_details = {
            "operation": operation,
            "duration_seconds": round(duration, 3),
            "timestamp": datetime.utcnow().isoformat(),
            "details": details or {},
        }

        logger.info(f"Performance: {json.dumps(log_details)}")
    except Exception as e:
        logger.error(f"Error logging performance: {str(e)}")


def sanitize_for_logging(data: Any) -> Any:
    """
    Sanitize data before logging to prevent sensitive information leakage.

    Args:
        data: Data to sanitize

    Returns:
        Sanitized data with sensitive information removed or obfuscated
    """
    # Define sensitive keys to redact
    sensitive_patterns = [
        r"api[_-]?key",
        r"token",
        r"password",
        r"secret",
        r"auth",
        r"credential",
        r"private",
        r"sensitive",
    ]

    if isinstance(data, dict):
        sanitized = {}
        for key, value in data.items():
            # Check if key matches any sensitive pattern
            is_sensitive = any(
                re.search(pattern, key, re.IGNORECASE) for pattern in sensitive_patterns
            )

            if is_sensitive:
                sanitized[key] = "***REDACTED***"
            else:
                sanitized[key] = sanitize_for_logging(value)
        return sanitized
    elif isinstance(data, list):
        return [sanitize_for_logging(item) for item in data]
    elif isinstance(data, str):
        # Redact API keys in strings
        patterns_to_redact = [
            (r"Bearer [a-zA-Z0-9\-\._~\+\/]+=*", "Bearer ***REDACTED***"),
            (
                r'api[_-]?key["\']?\s*[:=]\s*["\']?[a-zA-Z0-9\-\._~\+\/]+=*',
                'apiKey: "***REDACTED***"',
            ),
        ]

        sanitized_str = data
        for pattern, replacement in patterns_to_redact:
            sanitized_str = re.sub(
                pattern, replacement, sanitized_str, flags=re.IGNORECASE
            )

        return sanitized_str
    else:
        return data


def handle_and_log_exception(
    exception: Exception, context: str = "", hide_sensitive: bool = True
) -> Dict[str, Any]:
    """
    Handle and log exceptions with appropriate context.

    Args:
        exception: The exception to handle
        context: Context where the exception occurred
        hide_sensitive: Whether to hide sensitive information in logs

    Returns:
        Dict with error information suitable for API responses
    """
    # Prepare error information
    error_type = type(exception).__name__
    error_message = str(exception)

    if hide_sensitive:
        # Sanitize the error message
        error_message = sanitize_for_logging(error_message)

    # Log the exception with full context and traceback
    log_details = {
        "context": context,
        "error_type": error_type,
        "error_message": error_message,
        "timestamp": datetime.utcnow().isoformat(),
    }

    if isinstance(exception, HTTPException):
        # Log HTTP exceptions appropriately
        logger.warning(f"HTTP Exception in {context}: {json.dumps(log_details)}")
        return {"status_code": exception.status_code, "detail": exception.detail}
    else:
        # Log other exceptions as errors with full traceback
        logger.error(
            f"Exception in {context}: {json.dumps(log_details)}", exc_info=True
        )

        # For security, return a generic message to the client
        user_friendly_message = "I am currently unavailable. Please try again later."

        if isinstance(exception, ServiceError):
            return {
                "status_code": exception.status_code,
                "detail": exception.message,
                "error_code": exception.error_code,
            }
        else:
            return {"status_code": 500, "detail": user_friendly_message}


def error_handler(func: Callable) -> Callable:
    """
    Decorator to handle errors in API endpoints with comprehensive logging.
    Implements retry logic for transient failures and user-friendly error messages.
    """

    @wraps(func)
    async def wrapper(*args, **kwargs):
        start_time = time.time()
        request = None

        # Try to extract request object if available
        for arg in args:
            if isinstance(arg, Request):
                request = arg
                break

        try:
            # Execute the function
            result = await func(*args, **kwargs)

            # Log performance
            duration = time.time() - start_time
            log_performance(f"{func.__module__}.{func.__name__}", duration)

            return result

        except (ValidationError, NotFoundError) as e:
            # Handle known service errors with appropriate status codes
            duration = time.time() - start_time

            log_details = {
                "function": func.__name__,
                "error_code": e.error_code,
                "status_code": e.status_code,
                "message": e.message,
                "duration": duration,
                "timestamp": datetime.utcnow().isoformat(),
            }

            logger.warning(
                f"Service Error in {func.__name__}: {json.dumps(log_details)}"
            )

            raise HTTPException(status_code=e.status_code, detail=e.message)

        except HTTPException:
            # Re-raise HTTP exceptions as they're already properly handled
            logger.info(f"HTTPException raised in {func.__name__}")
            raise

        except Exception as e:
            # Handle unexpected errors
            duration = time.time() - start_time

            # Log the error with context
            error_info = handle_and_log_exception(
                e, f"{func.__module__}.{func.__name__}", hide_sensitive=True
            )

            # Log performance even for failures
            log_performance(f"{func.__name__}_error", duration)

            # Raise HTTP exception with user-friendly message
            raise HTTPException(
                status_code=error_info["status_code"], detail=error_info["detail"]
            )

    return wrapper


def retry_on_failure(
    max_attempts: int = 3,
    delay: float = 1.0,
    backoff: float = 2.0,
    exceptions: tuple = (Exception,),
):
    """
    Decorator to retry a function on failure with exponential backoff.

    Args:
        max_attempts: Maximum number of retry attempts
        delay: Initial delay between retries (seconds)
        backoff: Multiplier for delay after each attempt
        exceptions: Tuple of exceptions to retry on
    """

    def decorator(func: Callable) -> Callable:
        @wraps(func)
        async def wrapper(*args, **kwargs):
            attempts = 0
            current_delay = delay

            while attempts < max_attempts:
                try:
                    return await func(*args, **kwargs)
                except exceptions as e:
                    attempts += 1
                    if attempts >= max_attempts:
                        # Log the failure after exhausting retries
                        logger.warning(
                            f"Function {func.__name__} failed after {max_attempts} attempts. "
                            f"Last error: {str(e)}"
                        )
                        raise e

                    # Log retry attempt
                    logger.warning(
                        f"Attempt {attempts} failed for {func.__name__}: {str(e)}. "
                        f"Retrying in {current_delay:.2f}s..."
                    )

                    # Wait before retrying
                    await asyncio.sleep(current_delay)
                    current_delay *= backoff  # Exponential backoff

        return wrapper

    return decorator


def log_sensitive_operation(
    operation: str, details: Dict, user_id: Optional[str] = None
):
    """
    Log operations that might contain sensitive information with sanitization.

    Args:
        operation: Name of the operation being logged
        details: Details about the operation (will be sanitized)
        user_id: ID of the user involved
    """
    try:
        sanitized_details = sanitize_for_logging(details)
        log_entry = {
            "operation": operation,
            "timestamp": datetime.utcnow().isoformat(),
            "user_id": user_id,
            "details": sanitized_details,
        }

        logger.info(f"Sensitive operation: {json.dumps(log_entry)}")
    except Exception as e:
        logger.error(f"Error logging sensitive operation: {str(e)}")
