import asyncio
import time
from typing import Optional, Dict, Any
from datetime import datetime, timedelta
from schemas import CacheEntry


class TranslationCache:
    """
    Basic cache implementation for translation results
    Uses in-memory storage with TTL support
    Cache key format: {user_id}:{chapter_id}:{target_language}
    """

    def __init__(self):
        self._cache: Dict[str, CacheEntry] = {}
        self._lock = asyncio.Lock()

    def _generate_cache_key(self, user_id: str, chapter_id: str, target_language: str) -> str:
        """Generate cache key using user_id + chapter_id + target_language"""
        return f"{user_id}:{chapter_id}:{target_language}"

    async def get(self, user_id: str, chapter_id: str, target_language: str) -> Optional[CacheEntry]:
        """Get cached translation if it exists and hasn't expired"""
        key = self._generate_cache_key(user_id, chapter_id, target_language)

        async with self._lock:
            if key in self._cache:
                entry = self._cache[key]
                # Check if entry has expired
                if datetime.now() >= entry.expires_at:
                    # Remove expired entry
                    del self._cache[key]
                    return None
                return entry
            return None

    async def set(self, user_id: str, chapter_id: str, target_language: str, content: str, ttl_hours: int = 24) -> None:
        """Set cached translation with TTL (default 24 hours)"""
        key = self._generate_cache_key(user_id, chapter_id, target_language)

        expires_at = datetime.now() + timedelta(hours=ttl_hours)
        cache_entry = CacheEntry(
            user_id=user_id,
            chapter_id=chapter_id,
            target_language=target_language,
            content=content,
            created_at=datetime.now(),
            expires_at=expires_at
        )

        async with self._lock:
            self._cache[key] = cache_entry

    async def delete(self, user_id: str, chapter_id: str, target_language: str) -> bool:
        """Delete specific cached translation"""
        key = self._generate_cache_key(user_id, chapter_id, target_language)

        async with self._lock:
            if key in self._cache:
                del self._cache[key]
                return True
            return False

    async def clear_expired(self) -> int:
        """Clear all expired entries and return count of removed entries"""
        current_time = datetime.now()
        expired_keys = []

        async with self._lock:
            for key, entry in self._cache.items():
                if current_time >= entry.expires_at:
                    expired_keys.append(key)

            for key in expired_keys:
                del self._cache[key]

        return len(expired_keys)

    async def invalidate_user_cache(self, user_id: str) -> int:
        """Invalidate all cached translations for a specific user"""
        keys_to_remove = []

        async with self._lock:
            for key, entry in self._cache.items():
                if entry.user_id == user_id:
                    keys_to_remove.append(key)

            for key in keys_to_remove:
                del self._cache[key]

        return len(keys_to_remove)


# Global cache instance
translation_cache = TranslationCache()