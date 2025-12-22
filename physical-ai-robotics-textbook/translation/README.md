# Urdu Translation Service

This service provides AI-powered translation from English to Urdu with markdown preservation and caching capabilities.

## Features

- **Markdown Preservation**: Maintains all markdown formatting including headings, code blocks, lists, and links
- **Caching**: Implements 24-hour caching to improve performance and reduce API costs
- **Authentication**: Requires valid user authentication for translation access
- **RTL Support**: Proper right-to-left rendering for Urdu content
- **Error Handling**: Graceful fallback when translation service is unavailable
- **Large Content Support**: Chunking for handling large chapters

## API Endpoints

### POST `/api/translation/urdu`

Translates English content to Urdu with caching and authentication.

**Request Body**:
```json
{
  "chapter_id": "chapter-identifier",
  "source_language": "en",
  "target_language": "ur",
  "content": "# Markdown content to translate...",
  "user_id": "authenticated-user-id"
}
```

**Response**:
```json
{
  "translated_content": "# ترجمہ شدہ مواد...",
  "cached": true,
  "error": null
}
```

## Architecture

### Components

- `service.py`: Core translation logic with LLM integration and caching
- `router.py`: API endpoint definitions
- `cache.py`: In-memory caching with TTL support
- `auth.py`: Authentication validation utilities
- `schemas.py`: Pydantic models for request/response validation
- `markdown_utils.py`: Markdown structure preservation utilities
- `logging_config.py`: Structured logging for monitoring

### Flow

1. Authentication validation via dependency injection
2. Cache lookup using `user_id:chapter_id:target_language` key
3. If cached: return cached result
4. If not cached: call LLM service, preserve markdown structure, cache result
5. Return translated content

## Configuration

The service uses the following environment variables:
- `JWT_SECRET`: Secret key for JWT token validation (defaults to "your-secret-key-default")

## Performance

- Cached translations: < 200ms response time
- First-time translations: < 4 seconds
- Markdown structure preservation: > 95%

## Error Handling

- Translation service failures return original content with error notification
- Authentication failures return 401 Unauthorized
- Invalid requests return 400 Bad Request
- Internal errors return 500 Internal Server Error

## Security

- All endpoints require authentication
- User session validation on each request
- No original markdown files are modified
- Translations are derived views only