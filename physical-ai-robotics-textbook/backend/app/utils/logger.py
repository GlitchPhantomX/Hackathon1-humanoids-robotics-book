"""
Logging configuration for the RAG Chatbot API.
Sets up comprehensive logging as required by Phase 10.
"""

import logging
import sys
import os
from datetime import datetime
import json
from logging.handlers import RotatingFileHandler
from app.config import settings


def sanitize_for_logging(data):
    """
    Sanitize data before logging to prevent sensitive information leakage.

    Args:
        data: Data to sanitize

    Returns:
        Sanitized data with sensitive information removed or obfuscated
    """
    # Define sensitive keys to redact
    sensitive_patterns = [
        "api[_-]?key",
        "token",
        "password",
        "secret",
        "auth",
        "credential",
        "private",
        "sensitive",
        "google[_-]?api[_-]?key",
    ]

    if isinstance(data, dict):
        sanitized = {}
        for key, value in data.items():
            # Check if key matches any sensitive pattern
            is_sensitive = any(
                pattern.lower() in key.lower() for pattern in sensitive_patterns
            )

            if is_sensitive:
                sanitized[key] = "***REDACTED***"
            else:
                sanitized[key] = sanitize_for_logging(value)
        return sanitized
    elif isinstance(data, list):
        return [sanitize_for_logging(item) for item in data]
    elif isinstance(data, str):
        # Redact API keys and other sensitive info in strings
        import re

        sensitive_patterns_to_redact = [
            r"[a-zA-Z0-9]{39}",  # Generic API key pattern
            r"Bearer [a-zA-Z0-9\-\._~\+\/]+=*",  # Bearer tokens
            r"sk-[a-zA-Z0-9]+",  # OpenAI-style API keys
        ]

        sanitized_str = data
        for pattern in sensitive_patterns_to_redact:
            sanitized_str = re.sub(
                pattern, "***REDACTED***", sanitized_str, flags=re.IGNORECASE
            )

        return sanitized_str
    else:
        return data


def setup_logging():
    """
    Set up comprehensive logging for the application with:
    - Console output for development
    - File output for production
    - Log rotation (7-day retention)
    - Performance monitoring
    - Sensitive data protection
    """
    # Create logger
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    # Remove existing handlers to prevent duplicates
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)

    # Create formatters
    detailed_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s"
    )

    simple_formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")

    # Console handler for all environments
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(simple_formatter)
    logger.addHandler(console_handler)

    # File handler with rotation for production
    if hasattr(settings, "log_file_path") and settings.log_file_path:
        # Create log directory if it doesn't exist
        log_dir = os.path.dirname(settings.log_file_path)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        file_handler = RotatingFileHandler(
            settings.log_file_path,
            maxBytes=10 * 1024 * 1024,  # 10MB
            backupCount=7,  # Keep 7 files
            encoding="utf-8",
        )
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(detailed_formatter)
        logger.addHandler(file_handler)

    # Set specific log levels for different modules
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
    logging.getLogger("sqlalchemy.engine").setLevel(logging.WARNING)  # Reduce DB noise

    # Log initialization
    logger.info(f"Logging initialized at {datetime.utcnow().isoformat()}")
    logger.info(f"Log level set to: {logging.getLevelName(logger.level)}")

    return logger


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


def log_performance(operation: str, duration: float, details: dict = None):
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


def log_error(context: str, error: Exception, details: dict = None):
    """
    Log error with context and details.

    Args:
        context: Context where the error occurred
        error: The error that occurred
        details: Additional error details
    """
    try:
        log_details = {
            "context": context,
            "error_type": type(error).__name__,
            "error_message": str(error),
            "timestamp": datetime.utcnow().isoformat(),
            "details": details or {},
        }

        logger.error(f"Error: {json.dumps(log_details)}", exc_info=True)
    except Exception as e:
        logger.error(f"Error logging error (meta-error): {str(e)}", exc_info=True)


def log_sensitive_operation(operation: str, details: dict, user_id: str = None):
    """
    Log operations that contain sensitive information with data sanitization.

    Args:
        operation: Name of the sensitive operation
        details: Details about the operation (will be sanitized)
        user_id: User ID associated with the operation
    """
    try:
        # Sanitize sensitive information before logging
        sanitized_details = sanitize_for_logging(details)

        log_details = {
            "operation": operation,
            "timestamp": datetime.utcnow().isoformat(),
            "user_id": user_id,
            "details": sanitized_details,
        }

        logger.info(f"Sensitive operation: {json.dumps(log_details)}")
    except Exception as e:
        logger.error(f"Error logging sensitive operation: {str(e)}")


# Initialize the logger
logger = setup_logging()


def get_logger(name: str):
    """
    Get a named logger instance.

    Args:
        name: Name for the logger

    Returns:
        Configured logger instance
    """
    return logging.getLogger(name)


# Predefined loggers for common components
api_logger = get_logger("api")
db_logger = get_logger("database")
ai_logger = get_logger("ai_service")
perf_logger = get_logger("performance")
