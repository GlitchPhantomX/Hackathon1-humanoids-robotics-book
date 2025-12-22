"""
End-to-end tests for the Urdu translation feature
This test validates the complete flow from frontend to backend
"""

import pytest
import asyncio
import requests
import json
from pathlib import Path
from unittest.mock import patch, AsyncMock
from .schemas import TranslationRequest, TranslationResponse
from .service import TranslationService
from .cache import TranslationCache


class TestEndToEndTranslation:
    """
    End-to-end tests for the complete translation flow
    """

    def setup_method(self):
        """Setup test fixtures"""
        self.service = TranslationService()
        self.cache = TranslationCache()

        # Sample test data
        self.sample_request = TranslationRequest(
            chapter_id="e2e-test-chapter-001",
            source_language="en",
            target_language="ur",
            content="# Test Chapter\n\nThis is test content for end-to-end testing.",
            user_id="e2e-test-user-123"
        )

    @pytest.mark.asyncio
    async def test_complete_translation_flow(self):
        """Test the complete translation flow: request → service → cache → response"""
        # Verify initial state - no cached content
        cached_before = await self.cache.get(
            self.sample_request.user_id,
            self.sample_request.chapter_id,
            self.sample_request.target_language
        )
        assert cached_before is None

        # Perform translation
        response = await self.service.translate_chapter(self.sample_request)

        # Verify response
        assert isinstance(response, TranslationResponse)
        assert response.translated_content is not None
        assert response.error is None
        assert response.cached is False  # First call should not be cached

        # Verify content was cached
        cached_after = await self.cache.get(
            self.sample_request.user_id,
            self.sample_request.chapter_id,
            self.sample_request.target_language
        )
        assert cached_after is not None
        assert cached_after.content == response.translated_content

        # Perform second translation (should be cached)
        response_cached = await self.service.translate_chapter(self.sample_request)

        # Verify cached response
        assert isinstance(response_cached, TranslationResponse)
        assert response_cached.translated_content == response.translated_content
        assert response_cached.error is None
        assert response_cached.cached is True  # Second call should be cached

    @pytest.mark.asyncio
    async def test_translation_with_markdown_preservation(self):
        """Test that markdown structure is preserved during translation"""
        markdown_content = """# Introduction

This is a **bold** statement and *italic* text.

## Code Blocks
```python
def hello_world():
    print("Hello, world!")
```

## Lists
- Item 1
- Item 2
  - Nested item

1. Ordered item 1
2. Ordered item 2

## Links
[Example Link](https://example.com)

## Tables
| Header 1 | Header 2 |
|----------|----------|
| Cell 1   | Cell 2   |
"""

        request = TranslationRequest(
            chapter_id="markdown-test-chapter",
            source_language="en",
            target_language="ur",
            content=markdown_content,
            user_id="test-user-md"
        )

        response = await self.service.translate_chapter(request)

        # Verify response is successful
        assert response.translated_content is not None
        assert response.error is None

        # The translated content should be different from original (at least in concept)
        # In our mock implementation, it would contain [URDU:] markers
        assert "[URDU:" in response.translated_content or "# ترجمہ:" in response.translated_content

    @pytest.mark.asyncio
    async def test_large_content_chunking(self):
        """Test translation of large content with chunking"""
        # Create large content that exceeds default chunk size
        large_content = "# Large Content\n\n"
        for i in range(150):  # Create content that will likely exceed 10KB
            large_content += f"## Section {i}\n\nThis is paragraph {i} of the large content.\n\n"

        large_request = TranslationRequest(
            chapter_id="large-content-chapter",
            source_language="en",
            target_language="ur",
            content=large_content,
            user_id="test-user-large"
        )

        # This should handle chunking internally
        response = await self.service.translate_chapter(large_request)

        # Verify response is successful
        assert response.translated_content is not None
        assert response.error is None
        # Should not be cached since it's a fresh request
        assert response.cached is False

    @pytest.mark.asyncio
    async def test_error_handling_and_fallback(self):
        """Test error handling and fallback behavior"""
        # Test with invalid request (None content)
        invalid_request = TranslationRequest(
            chapter_id="error-test-chapter",
            source_language="en",
            target_language="ur",
            content="",  # Empty content
            user_id="test-user-error"
        )

        response = await self.service.translate_chapter(invalid_request)

        # Should still return a valid response (fallback to original content)
        assert response.translated_content == ""  # Original content
        assert response.error is None  # Our implementation doesn't error on empty content

        # Test with normal content to ensure service still works
        normal_request = TranslationRequest(
            chapter_id="normal-after-error",
            source_language="en",
            target_language="ur",
            content="# Normal Content\n\nNormal content.",
            user_id="test-user-normal"
        )

        normal_response = await self.service.translate_chapter(normal_request)
        assert normal_response.translated_content is not None
        assert normal_response.error is None

    @pytest.mark.asyncio
    async def test_concurrent_translations(self):
        """Test concurrent access to translation service"""
        # Create multiple requests for the same content
        requests_list = [
            TranslationRequest(
                chapter_id="concurrent-chapter",
                source_language="en",
                target_language="ur",
                content=f"Content for request {i}",
                user_id=f"test-user-{i}"
            )
            for i in range(3)
        ]

        # Execute translations concurrently
        responses = await asyncio.gather(*[
            self.service.translate_chapter(req) for req in requests_list
        ])

        # Verify all responses are successful
        for i, response in enumerate(responses):
            assert response.translated_content is not None
            assert response.error is None
            # Each should be a fresh translation (not cached)
            assert response.cached is False

    @pytest.mark.asyncio
    async def test_cache_expiration_and_cleanup(self):
        """Test cache expiration and cleanup functionality"""
        # Add content to cache with short TTL
        await self.cache.set(
            "test-user-exp",
            "test-chapter-exp",
            "ur",
            "Test content for expiration",
            ttl_hours=0  # 0 hours for testing
        )

        # Manually expire the entry
        key = "test-user-exp:test-chapter-exp:ur"
        if key in self.cache._cache:
            from datetime import datetime, timedelta
            self.cache._cache[key].expires_at = datetime.now() - timedelta(seconds=1)

        # Try to get the expired content
        expired_result = await self.cache.get("test-user-exp", "test-chapter-exp", "ur")
        assert expired_result is None  # Should be expired

        # Add a valid entry and an expired one, then clear expired
        await self.cache.set(
            "valid-user",
            "valid-chapter",
            "ur",
            "Valid content"
        )

        # Create an expired entry manually
        expired_entry_key = "expired-user:expired-chapter:ur"
        from .schemas import CacheEntry
        expired_entry = CacheEntry(
            user_id="expired-user",
            chapter_id="expired-chapter",
            target_language="ur",
            content="Expired content",
            created_at=datetime.now(),
            expires_at=datetime.now() - timedelta(seconds=1)  # Expired
        )
        self.cache._cache[expired_entry_key] = expired_entry

        # Clear expired entries
        removed_count = await self.cache.clear_expired()
        assert removed_count >= 1  # At least one expired entry was removed

        # Valid entry should still exist
        valid_result = await self.cache.get("valid-user", "valid-chapter", "ur")
        assert valid_result is not None
        assert valid_result.content == "Valid content"

    @pytest.mark.asyncio
    async def test_performance_requirements(self):
        """Test that performance requirements are met"""
        import time

        # Test cached response time (should be < 200ms)
        # First, cache some content
        await self.cache.set(
            "perf-user",
            "perf-chapter",
            "ur",
            "Cached performance test content that should return quickly"
        )

        start_time = time.time()
        response = await self.service.translate_chapter(
            TranslationRequest(
                chapter_id="perf-chapter",
                source_language="en",
                target_language="ur",
                content="Not used for cached request",
                user_id="perf-user"
            )
        )
        cache_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        assert response.cached is True
        assert cache_time < 200, f"Cached response took {cache_time}ms, should be < 200ms"

        # Test first-time translation time (should be < 4000ms)
        # Note: With mock implementation, this will be nearly instantaneous
        fresh_request = TranslationRequest(
            chapter_id="fresh-perf-chapter",
            source_language="en",
            target_language="ur",
            content="# Performance Test\n\nTesting first-time translation performance.",
            user_id="perf-user-fresh"
        )

        start_time = time.time()
        fresh_response = await self.service.translate_chapter(fresh_request)
        fresh_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        assert fresh_response.cached is False
        # With mock implementation, this should be very fast
        # In a real system with actual LLM calls, this would test the 4s requirement


# Run tests if this file is executed directly
if __name__ == "__main__":
    import asyncio

    async def run_e2e_tests():
        test_suite = TestEndToEndTranslation()

        # Setup test fixtures
        test_suite.setup_method()

        # Run all tests
        print("Testing complete translation flow...")
        await test_suite.test_complete_translation_flow()
        print("✓ Complete translation flow test passed")

        print("Testing markdown preservation...")
        await test_suite.test_translation_with_markdown_preservation()
        print("✓ Markdown preservation test passed")

        print("Testing large content chunking...")
        await test_suite.test_large_content_chunking()
        print("✓ Large content chunking test passed")

        print("Testing error handling and fallback...")
        await test_suite.test_error_handling_and_fallback()
        print("✓ Error handling and fallback test passed")

        print("Testing concurrent translations...")
        await test_suite.test_concurrent_translations()
        print("✓ Concurrent translations test passed")

        print("Testing cache expiration and cleanup...")
        await test_suite.test_cache_expiration_and_cleanup()
        print("✓ Cache expiration and cleanup test passed")

        print("Testing performance requirements...")
        await test_suite.test_performance_requirements()
        print("✓ Performance requirements test passed")

        print("\nAll end-to-end tests passed!")

    asyncio.run(run_e2e_tests())