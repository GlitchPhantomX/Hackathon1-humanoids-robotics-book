# Data Model: Urdu Translation Toggle

## Entities

### Translation Request
- **Fields**:
  - `chapter_id` (string): Unique identifier for the chapter being translated
  - `source_language` (string): Source language code (e.g., "en")
  - `target_language` (string): Target language code (e.g., "ur")
  - `content` (string): Original markdown content to translate
  - `user_id` (string): Identifier of requesting user (for caching)
- **Validation**: All fields required, language codes must be valid

### Translation Response
- **Fields**:
  - `translated_content` (string): Translated markdown content
  - `cached` (boolean): Whether response came from cache
  - `error` (string, optional): Error message if translation failed
- **Validation**: translated_content required when no error

### Cache Entry
- **Fields**:
  - `user_id` (string): User identifier
  - `chapter_id` (string): Chapter identifier
  - `target_language` (string): Target language code
  - `content` (string): Cached translated content
  - `created_at` (timestamp): When cache entry was created
  - `expires_at` (timestamp): When cache entry expires (24 hours from creation)
- **Validation**: TTL set to 24 hours as per specification

### User Session
- **Fields**:
  - `user_id` (string): Unique user identifier
  - `authenticated` (boolean): Whether user is currently authenticated
  - `session_token` (string): Token for session validation
- **Validation**: Session must be valid for translation requests