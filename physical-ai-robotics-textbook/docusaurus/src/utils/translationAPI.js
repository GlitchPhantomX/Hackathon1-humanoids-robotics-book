const TRANSLATION_API_URL = 'http://localhost:8001';

export async function translateContent(content, chapterId, userId = 'local-user') {
  try {
    const response = await fetch(`${TRANSLATION_API_URL}/api/translation/urdu`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        user_id: userId,
        chapter_id: chapterId,
        content: content,
        source_language: 'en',
        target_language: 'ur',
      }),
    });

    if (!response.ok) {
      throw new Error(`Translation failed: ${response.status}`);
    }

    const data = await response.json();
    return data.translated_content;
  } catch (error) {
    console.error('Translation API Error:', error);
    return content; // Fallback to original content
  }
}