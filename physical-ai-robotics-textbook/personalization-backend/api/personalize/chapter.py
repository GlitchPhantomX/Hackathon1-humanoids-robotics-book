from fastapi import APIRouter, HTTPException, Depends, Request
import asyncio
import time
import logging
from typing import Dict, Any, Optional
from collections import defaultdict
from datetime import datetime, timedelta

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from models.personalization import PersonalizeChapterRequest, PersonalizeChapterResponse, ErrorResponse
from services.ai_personalization import ai_personalization_service
from services.markdown_chunker import MarkdownChunker
from services.error_handler import PersonalizationErrorHandler

router = APIRouter(prefix="/api/personalize", tags=["personalization"])

logger = logging.getLogger(__name__)

rate_limit_storage = defaultdict(list)
RATE_LIMIT_WINDOW = 60
RATE_LIMIT_MAX_REQUESTS = 10


def check_rate_limit(user_id: str) -> bool:
    """Check if user has exceeded rate limit"""
    now = datetime.now()
    window_start = now - timedelta(seconds=RATE_LIMIT_WINDOW)

    rate_limit_storage[user_id] = [
        timestamp for timestamp in rate_limit_storage[user_id]
        if timestamp > window_start
    ]

    if len(rate_limit_storage[user_id]) >= RATE_LIMIT_MAX_REQUESTS:
        return False

    rate_limit_storage[user_id].append(now)
    return True


@router.post("/chapter", response_model=PersonalizeChapterResponse)
async def personalize_chapter(
    request: PersonalizeChapterRequest,
):
    """
    Personalize chapter content based on user profile
    """
    start_time = time.time()
    user_id = "test_user"
    
    # ✅ ADDED: More detailed logging
    logger.info(f"📥 Received personalization request")
    logger.info(f"📄 Chapter ID: {request.chapter_id}")
    logger.info(f"📝 Content length: {len(request.chapter_content)} chars")
    logger.info(f"👤 User level: {request.user_profile.roboticsExperience}")

    # Rate limiting
    if not check_rate_limit(user_id):
        logger.warning(f"Rate limit exceeded for user: {user_id}")
        raise HTTPException(status_code=429, detail="Rate limit exceeded. Please try again later.")

    try:
        content_length = len(request.chapter_content)

        # ✅ REMOVED TIMEOUT CHECK - Let it process fully
        # Mock personalization takes 3 seconds, so we need more time

        # Invoke AI personalization service
        chunk_start_time = time.time()
        
        if content_length > 5000:  # ✅ Increased threshold
            logger.info(f"🔄 Using chunked processing for large content ({content_length} chars)")
            sections = MarkdownChunker.split_by_headers(request.chapter_content)
            logger.info(f"📑 Content split into {len(sections)} sections")

            personalized_sections = await ai_personalization_service.personalize_content_sections(
                sections,
                request.user_profile
            )
            personalized_content = "\n\n".join(personalized_sections)

            chunk_processing_time = (time.time() - chunk_start_time) * 1000
            logger.info(f"⏱️ Chunked processing time: {chunk_processing_time:.2f}ms for {len(sections)} sections")
        else:
            logger.info(f"🔄 Using single-block processing for content ({content_length} chars)")
            
            # ✅ CALL PERSONALIZATION SERVICE
            personalized_content = await ai_personalization_service.personalize_content(
                request.chapter_content,
                request.user_profile
            )

            single_processing_time = (time.time() - chunk_start_time) * 1000
            logger.info(f"⏱️ Single-block processing time: {single_processing_time:.2f}ms")

        # ✅ VERIFY CONTENT WAS PERSONALIZED
        if len(personalized_content) < 10:
            logger.error(f"❌ Personalized content too short: {len(personalized_content)} chars")
            personalized_content = request.chapter_content

        total_processing_time = int((time.time() - start_time) * 1000)
        logger.info(f"✅ Personalization completed: {len(request.chapter_content)} → {len(personalized_content)} chars in {total_processing_time}ms")

        response = PersonalizeChapterResponse(
            personalized_content=personalized_content,
            processing_time_ms=total_processing_time
        )

        return response

    except HTTPException as http_exc:
        error_time = (time.time() - start_time) * 1000
        logger.error(f"❌ HTTP error: {http_exc.detail}, time: {error_time:.2f}ms")
        raise
    except Exception as e:
        error_time = int((time.time() - start_time) * 1000)
        logger.error(f"❌ Unexpected error: {str(e)}, time: {error_time}ms", exc_info=True)

        # Fallback to original content
        return PersonalizeChapterResponse(
            personalized_content=request.chapter_content,
            processing_time_ms=error_time
        )


@router.get("/health")
async def health_check():
    """Simple health check endpoint"""
    return {"status": "healthy", "service": "personalization-api"}