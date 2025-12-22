from fastapi import APIRouter, HTTPException, status, Request
import logging
from schemas import TranslationRequest, TranslationResponse
from service import translation_service
from cache import translation_cache

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["translation"])


@router.get("/test")
async def test():
    return {"status": "ok", "message": "Router is working!"}


@router.post("/translation/urdu", response_model=TranslationResponse)
async def translate_to_urdu(request: TranslationRequest):
    """
    API endpoint to translate content to Urdu
    """
    logger.info("=" * 50)
    logger.info("🎯 TRANSLATION REQUEST RECEIVED!")
    logger.info(f"Chapter: {request.chapter_id}")
    logger.info(f"Content length: {len(request.content)} chars")
    logger.info("=" * 50)
    
    try:
        logger.info(f"Received translation request for chapter {request.chapter_id}")

        if not request.chapter_id or not request.content:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="chapter_id and content are required"
            )

        result = await translation_service.translate_chapter(request)
        logger.info(f"Translation completed for chapter {request.chapter_id}")

        return result

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Translation API error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation service error: {str(e)}"
        )


@router.get("/translation/cache/status")
async def get_cache_status():
    return {
        "message": "Cache system operational",
        "cache_size": len(translation_cache._cache) if hasattr(translation_cache, '_cache') else 0
    }


@router.delete("/translation/cache/{user_id}")
async def clear_user_cache(user_id: str):
    try:
        removed_count = await translation_cache.invalidate_user_cache(user_id)
        return {
            "message": f"Cleared {removed_count} cached translations for user {user_id}",
            "removed_count": removed_count
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error clearing cache: {str(e)}"
        )

# ✅ FILE ENDS HERE - NO MORE CODE!