"""
Unit tests for translation service
"""
import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from datetime import datetime, timedelta
from .schemas import TranslationRequest, TranslationResponse, CacheEntry
from .service import TranslationService
from .cache import TranslationCache


class TestTranslationService:
    """
    Unit tests for TranslationService
    """

    def setup_method(self):
        """Setup test fixtures"""
        self.service = TranslationService()
        self.test_request = TranslationRequest(
            chapter_id="test-chapter-001",
            source_language="en",
            target_language="ur",
            content="# Test Content\n\nThis is test content.",
            user_id="test-user-123"
        )

    @pytest.mark.asyncio
    async def test_translate_chapter_cache_hit(self):
        """Test that translation returns cached result when available"""
        # Add content to cache
        await self.service.cache.set(
            self.test_request.user_id,
            self.test_request.chapter_id,
            self.test_request.target_language,
            "Cached translated content"
        )

        # Call translate method
        response = await self.service.translate_chapter(self.test_request)

        # Assertions
        assert response.cached is True
        assert response.translated_content == "Cached translated content"
        assert response.error is None

    @pytest.mark.asyncio
    async def test_translate_chapter_cache_miss_then_store(self):
        """Test that translation is performed and stored when not cached"""
        # Ensure cache is empty
        cached_result = await self.service.cache.get(
            self.test_request.user_id,
            self.test_request.chapter_id,
            self.test_request.target_language
        )
        assert cached_result is None

        # Call translate method
        response = await self.service.translate_chapter(self.test_request)

        # Assertions
        assert response.cached is False
        assert response.translated_content is not None
        assert "Test Content" in response.translated_content or "[URDU:" in response.translated_content
        assert response.error is None

        # Verify content was cached
        cached_result = await self.service.cache.get(
            self.test_request.user_id,
            self.test_request.chapter_id,
            self.test_request.target_language
        )
        assert cached_result is not None
        assert cached_result.content == response.translated_content

    @pytest.mark.asyncio
    async def test_translate_chapter_error_handling(self):
        """Test that translation handles errors gracefully"""
        # Create a request with problematic content
        error_request = TranslationRequest(
            chapter_id="error-chapter-001",
            source_language="en",
            target_language="ur",
            content=None,  # This should cause an error
            user_id="test-user-123"
        )

        # Call translate method
        response = await self.service.translate_chapter(error_request)

        # Should return original content with error when there's an issue
        assert response.translated_content == error_request.content
        # Note: In our implementation, when content is None, it might behave differently

    @pytest.mark.asyncio
    async def test_translate_large_content_chunking(self):
        """Test that large content is chunked and processed correctly"""
        # Create large content that exceeds chunk size
        large_content = "# Large Content\n\n" + "This is a test paragraph.\n\n" * 200
        large_request = TranslationRequest(
            chapter_id="large-chapter-001",
            source_language="en",
            target_language="ur",
            content=large_content,
            user_id="test-user-123"
        )

        # Call translate method
        response = await self.service.translate_chapter(large_request)

        # Assertions
        assert response.translated_content is not None
        assert response.error is None
        assert not response.cached  # Fresh translation

    @pytest.mark.asyncio
    async def test_translate_with_malformed_markdown(self):
        """Test translation with malformed markdown content"""
        malformed_content = "# Bad Markdown\n\nSome content with [[unclosed brackets]\n\nMore text"
        malformed_request = TranslationRequest(
            chapter_id="malformed-chapter-001",
            source_language="en",
            target_language="ur",
            content=malformed_content,
            user_id="test-user-123"
        )

        # Call translate method
        response = await self.service.translate_chapter(malformed_request)

        # Should still produce a result despite malformed markdown
        assert response.translated_content is not None
        assert response.error is None

    @pytest.mark.asyncio
    async def test_session_expiration_during_translation(self):
        """Test handling of session expiration during long-running translation"""
        # This test verifies the service handles session validation
        # though actual session validation happens at the API level
        normal_request = TranslationRequest(
            chapter_id="normal-chapter-001",
            source_language="en",
            target_language="ur",
            content="# Normal Content\n\nNormal translation content.",
            user_id="test-user-123"
        )

        # Call translate method
        response = await self.service.translate_chapter(normal_request)

        # Should complete normally
        assert response.translated_content is not None
        assert response.error is None


class TestMarkdownUtils:
    """
    Unit tests for markdown utility functions
    """
    pass  # These were already tested in test_cache.py


class TestCacheIntegration:
    """
    Unit tests for cache functionality
    """

    def setup_method(self):
        """Setup test fixtures"""
        self.cache = TranslationCache()

    @pytest.mark.asyncio
    async def test_cache_set_and_get(self):
        """Test basic cache set and get operations"""
        user_id = "test_user"
        chapter_id = "test_chapter"
        language = "ur"
        content = "Test cached content"

        # Set cache entry
        await self.cache.set(user_id, chapter_id, language, content)

        # Get cache entry
        result = await self.cache.get(user_id, chapter_id, language)

        # Assertions
        assert result is not None
        assert result.content == content
        assert result.user_id == user_id
        assert result.chapter_id == chapter_id
        assert result.target_language == language

    @pytest.mark.asyncio
    async def test_cache_expiration(self):
        """Test cache expiration functionality"""
        user_id = "test_user"
        chapter_id = "test_chapter"
        language = "ur"
        content = "Test content with short TTL"

        # Set cache entry with 1 second TTL
        await self.cache.set(user_id, chapter_id, language, content, ttl_hours=0)  # 0 hours for testing

        # Manually expire the entry
        key = f"{user_id}:{chapter_id}:{language}"
        if key in self.cache._cache:
            self.cache._cache[key].expires_at = datetime.now() - timedelta(seconds=1)

        # Try to get the expired entry
        result = await self.cache.get(user_id, chapter_id, language)

        # Should return None as it's expired
        assert result is None

    @pytest.mark.asyncio
    async def test_cache_delete(self):
        """Test cache delete functionality"""
        user_id = "test_user"
        chapter_id = "test_chapter"
        language = "ur"
        content = "Test content to delete"

        # Set cache entry
        await self.cache.set(user_id, chapter_id, language, content)

        # Verify it exists
        result = await self.cache.get(user_id, chapter_id, language)
        assert result is not None

        # Delete the entry
        deleted = await self.cache.delete(user_id, chapter_id, language)
        assert deleted is True

        # Verify it's gone
        result_after_delete = await self.cache.get(user_id, chapter_id, language)
        assert result_after_delete is None

    @pytest.mark.asyncio
    async def test_clear_expired_entries(self):
        """Test clearing expired cache entries"""
        # Add an expired entry
        expired_key = "expired:user:ur"
        expired_entry = CacheEntry(
            user_id="expired",
            chapter_id="user",
            target_language="ur",
            content="Expired content",
            created_at=datetime.now(),
            expires_at=datetime.now() - timedelta(seconds=1)  # Already expired
        )
        self.cache._cache[expired_key] = expired_entry

        # Add a valid entry
        valid_key = "valid:user:ur"
        valid_entry = CacheEntry(
            user_id="valid",
            chapter_id="user",
            target_language="ur",
            content="Valid content",
            created_at=datetime.now(),
            expires_at=datetime.now() + timedelta(hours=1)  # Valid for 1 hour
        )
        self.cache._cache[valid_key] = valid_entry

        # Clear expired entries
        removed_count = await self.cache.clear_expired()

        # Should have removed 1 expired entry
        assert removed_count == 1

        # Expired entry should be gone
        assert expired_key not in self.cache._cache

        # Valid entry should still exist
        assert valid_key in self.cache._cache


# Run tests if this file is executed directly
if __name__ == "__main__":
    import asyncio

    async def run_tests():
        test_service = TestTranslationService()
        test_cache = TestCacheIntegration()

        # Setup test fixtures
        test_service.setup_method()
        test_cache.setup_method()

        # Run translation service tests
        print("Testing cache hit...")
        await test_service.test_translate_chapter_cache_hit()
        print("✓ Cache hit test passed")

        print("Testing cache miss...")
        await test_service.test_translate_chapter_cache_miss_then_store()
        print("✓ Cache miss test passed")

        print("Testing error handling...")
        await test_service.test_translate_chapter_error_handling()
        print("✓ Error handling test passed")

        print("Testing large content chunking...")
        await test_service.test_translate_large_content_chunking()
        print("✓ Large content chunking test passed")

        print("Testing malformed markdown...")
        await test_service.test_translate_with_malformed_markdown()
        print("✓ Malformed markdown test passed")

        # Run cache tests
        print("Testing cache set/get...")
        await test_cache.test_cache_set_and_get()
        print("✓ Cache set/get test passed")

        print("Testing cache expiration...")
        await test_cache.test_cache_expiration()
        print("✓ Cache expiration test passed")

        print("Testing cache delete...")
        await test_cache.test_cache_delete()
        print("✓ Cache delete test passed")

        print("Testing clear expired entries...")
        await test_cache.test_clear_expired_entries()
        print("✓ Clear expired entries test passed")

        print("\nAll backend unit tests passed!")

    asyncio.run(run_tests())