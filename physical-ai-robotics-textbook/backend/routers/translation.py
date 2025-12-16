from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import openai

router = APIRouter(prefix="/api/translation", tags=["translation"])

class TranslationRequest(BaseModel):
    content: str
    source_lang: str = "en"
    target_lang: str
    chapter_id: str

@router.post("/translate")
async def translate_content(request: TranslationRequest):
    """Translate content using OpenAI GPT-4"""
    try:
        prompt = f"""
        Translate this educational content from {request.source_lang} to {request.target_lang}.

        CRITICAL: Preserve ALL HTML tags, CSS classes, and IDs exactly.
        Only translate text content.
        Keep code blocks in English.

        Content:
        {request.content}
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "Professional technical translator"},
                {"role": "user", "content": prompt}
            ],
            temperature=0.3
        )

        return {
            "success": True,
            "translated_content": response.choices[0].message.content,
            "target_lang": request.target_lang
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))