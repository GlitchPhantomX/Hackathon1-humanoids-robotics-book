"""
Logging configuration for translation service
"""
import logging
import sys
from datetime import datetime
from typing import Optional


class TranslationLogger:
    """
    Dedicated logger for translation service with structured logging
    """

    def __init__(self, name: str = "translation-service"):
        self.logger = logging.getLogger(name)

        # Prevent duplicate handlers if logger already configured
        if not self.logger.handlers:
            self.logger.setLevel(logging.INFO)

            # Create console handler
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(logging.INFO)

            # Create file handler
            file_handler = logging.FileHandler("translation_service.log")
            file_handler.setLevel(logging.INFO)

            # Create formatter
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            console_handler.setFormatter(formatter)
            file_handler.setFormatter(formatter)

            # Add handlers to logger
            self.logger.addHandler(console_handler)
            self.logger.addHandler(file_handler)

            # Prevent propagation to root logger to avoid duplicate logs
            self.logger.propagate = False

    def log_translation_request(self, user_id: str, chapter_id: str, target_language: str):
        """Log translation request"""
        self.logger.info(
            f"TRANSLATION_REQUEST - user_id: {user_id}, "
            f"chapter_id: {chapter_id}, target_language: {target_language}"
        )

    def log_translation_success(self, user_id: str, chapter_id: str, target_language: str,
                               response_time_ms: float, cached: bool):
        """Log successful translation"""
        cache_status = "CACHED" if cached else "FRESH"
        self.logger.info(
            f"TRANSLATION_SUCCESS - {cache_status} - user_id: {user_id}, "
            f"chapter_id: {chapter_id}, target_language: {target_language}, "
            f"response_time: {response_time_ms:.2f}ms"
        )

    def log_translation_failure(self, user_id: str, chapter_id: str, target_language: str,
                                error_message: str, error_type: str = "GENERAL"):
        """Log translation failure with detailed error information"""
        self.logger.error(
            f"TRANSLATION_FAILURE - {error_type} - user_id: {user_id}, "
            f"chapter_id: {chapter_id}, target_language: {target_language}, "
            f"error: {error_message}"
        )

    def log_cache_operation(self, operation: str, user_id: str, chapter_id: str,
                           target_language: str, hit: Optional[bool] = None):
        """Log cache operations"""
        if hit is not None:
            hit_status = "HIT" if hit else "MISS"
            self.logger.info(
                f"CACHE_{operation.upper()} - {hit_status} - user_id: {user_id}, "
                f"chapter_id: {chapter_id}, target_language: {target_language}"
            )
        else:
            self.logger.info(
                f"CACHE_{operation.upper()} - user_id: {user_id}, "
                f"chapter_id: {chapter_id}, target_language: {target_language}"
            )

    def log_authentication_check(self, user_id: str, ip_address: Optional[str] = None):
        """Log authentication checks"""
        ip_part = f", ip: {ip_address}" if ip_address else ""
        self.logger.info(
            f"AUTH_CHECK - user_id: {user_id}{ip_part}"
        )

    def log_session_expiration(self, user_id: str, token: str):
        """Log session expiration events"""
        self.logger.warning(
            f"SESSION_EXPIRED - user_id: {user_id}, token_expired: {token[:10]}..."
        )


# Global logger instance
translation_logger = TranslationLogger()