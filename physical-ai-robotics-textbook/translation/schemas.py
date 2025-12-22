from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime
import re
from pydantic import field_validator


class TranslationRequest(BaseModel):
    """
    Schema for translation requests
    Fields:
    - chapter_id (string): Unique identifier for the chapter being translated
    - source_language (string): Source language code (e.g., "en")
    - target_language (string): Target language code (e.g., "ur")
    - content (string): Original markdown content to translate
    - user_id (string): Identifier of requesting user (for caching)
    Validation: All fields required, language codes must be valid
    """
    chapter_id: str = Field(..., description="Unique identifier for the chapter being translated")
    source_language: str = Field(..., description="Source language code")
    target_language: str = Field(..., description="Target language code")
    content: str = Field(..., description="Original markdown content to translate")
    user_id: str = Field(..., description="Identifier of requesting user (for caching)")

    @field_validator('source_language', 'target_language')
    @classmethod
    def validate_language_code(cls, v):
        if len(v) != 2 or not v.isalpha():
            raise ValueError('Language code must be 2 alphabetic characters')
        return v.lower()


class TranslationResponse(BaseModel):
    """
    Schema for translation responses
    Fields:
    - translated_content (string): Translated markdown content
    - cached (boolean): Whether response came from cache
    - error (string, optional): Error message if translation failed
    Validation: translated_content required when no error
    """
    translated_content: Optional[str] = Field(None, description="Translated markdown content")
    cached: bool = Field(default=False, description="Whether response came from cache")
    error: Optional[str] = Field(None, description="Error message if translation failed")

    def __init__(self, **data):
        super().__init__(**data)
        # Validate that translated_content is provided when no error occurred
        if not self.error and not self.translated_content:
            raise ValueError('translated_content is required when no error occurred')


class CacheEntry(BaseModel):
    """
    Schema for cache entries
    Fields:
    - user_id (string): User identifier
    - chapter_id (string): Chapter identifier
    - target_language (string): Target language code
    - content (string): Cached translated content
    - created_at (timestamp): When cache entry was created
    - expires_at (timestamp): When cache entry expires (24 hours from creation)
    Validation: TTL set to 24 hours as per specification
    """
    user_id: str
    chapter_id: str
    target_language: str
    content: str
    created_at: datetime
    expires_at: datetime


class UserSession(BaseModel):
    """
    Schema for user session information
    Fields:
    - user_id (string): Unique user identifier
    - authenticated (boolean): Whether user is currently authenticated
    - session_token (string): Token for session validation
    Validation: Session must be valid for translation requests
    """
    user_id: str
    authenticated: bool
    session_token: Optional[str] = None