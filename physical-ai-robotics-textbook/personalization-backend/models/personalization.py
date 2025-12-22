from pydantic import BaseModel, Field
from typing import Optional


class UserProfile(BaseModel):
    """
    Model representing user profile data from Better Auth
    """
    softwareBackground: str = Field(
        ...,
        description="User's software background level",
        min_length=1,
        max_length=50,
        pattern=r"^[a-zA-Z0-9\s\-_]+$"
    )
    hardwareBackground: str = Field(
        ...,
        description="User's hardware background",
        min_length=1,
        max_length=50,
        pattern=r"^[a-zA-Z0-9\s\-_]+$"
    )
    programmingLanguages: str = Field(
        ...,
        description="Programming languages the user knows",
        min_length=1,
        max_length=200,
        pattern=r"^[a-zA-Z0-9\s,\-+_]+$"
    )
    roboticsExperience: str = Field(
        ...,
        description="User's robotics experience level",
        min_length=1,
        max_length=50,
        pattern=r"^[a-zA-Z0-9\s\-_]+$"
    )
    aiMlExperience: str = Field(
        ...,
        description="User's AI/ML experience level",
        min_length=1,
        max_length=50,
        pattern=r"^[a-zA-Z0-9\s\-_]+$"
    )
    hasRosExperience: bool = Field(
        ...,
        description="Whether the user has ROS experience"
    )
    hasGpuAccess: bool = Field(
        ...,
        description="Whether the user has GPU access"
    )
    learningGoals: str = Field(
        ...,
        description="User's learning goals",
        min_length=1,
        max_length=500,
        pattern=r"^[a-zA-Z0-9\s\-_.,!?]+$"
    )


class PersonalizeChapterRequest(BaseModel):
    """
    Request model for personalizing a chapter
    """
    # ✅ Add aliases to accept both camelCase and snake_case
    chapter_id: str = Field(
        ...,
        alias="chapterId",  # Accept "chapterId" from frontend
        description="Unique identifier for the chapter",
        min_length=1,
        max_length=100,
        # ✅ Relaxed pattern to accept paths like /docs/chapter-1
        pattern=r"^[a-zA-Z0-9\-_/]+$"
    )
    chapter_content: str = Field(
        ...,
        alias="chapterContent",  # Accept "chapterContent" from frontend
        description="Markdown content of the chapter to be personalized",
        min_length=1,
        max_length=50000
    )
    user_profile: UserProfile = Field(
        ...,
        alias="userProfile"  # Accept "userProfile" from frontend
    )
    
    class Config:
        populate_by_name = True  # ✅ Allow both camelCase and snake_case


class PersonalizeChapterResponse(BaseModel):
    """
    Response model for personalized chapter content
    """
    personalized_content: str = Field(..., alias="personalizedContent")
    processing_time_ms: int = Field(..., alias="processingTimeMs")
    section_id: Optional[str] = Field(None, alias="sectionId")
    
    class Config:
        populate_by_name = True


class ErrorResponse(BaseModel):
    """
    Standardized error response model
    """
    error: str
    fallback_content: Optional[str] = Field(None, alias="fallbackContent")
    code: str
    
    class Config:
        populate_by_name = True