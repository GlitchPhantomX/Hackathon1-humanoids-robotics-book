import asyncio
import logging
from typing import Optional
from schemas import TranslationRequest, TranslationResponse
import re

logging.basicConfig(level=logging.INFO, format='%(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class TranslationService:
    def __init__(self):
        self.model_loaded = False
        logger.info("🔧 Loading translation model...")
        
        try:
            from transformers import MarianMTModel, MarianTokenizer
            import torch
            
            model_name = "Helsinki-NLP/opus-mt-en-ur"
            self.tokenizer = MarianTokenizer.from_pretrained(model_name)
            self.model = MarianMTModel.from_pretrained(model_name)
            self.model.eval()
            self.model_loaded = True
            logger.info("✅ Model ready!")
            
        except Exception as e:
            logger.error(f"❌ Failed: {e}")

    async def translate_chapter(self, request: TranslationRequest) -> TranslationResponse:
        try:
            logger.info(f"🌐 Translating: {request.chapter_id}")
            
            if not self.model_loaded:
                return TranslationResponse(
                    translated_content=request.content,
                    chapter_id=request.chapter_id,
                    cached=False
                )
            
            # ✅ Process line by line, skip code blocks
            lines = request.content.split('\n')
            translated = []
            in_code_block = False
            
            for line in lines:
                # Detect code block markers
                if '```' in line or line.strip().startswith('graph '):
                    in_code_block = not in_code_block
                    translated.append(line)  # Keep as-is
                    continue
                
                # If inside code block, don't translate
                if in_code_block:
                    translated.append(line)
                    continue
                
                # Skip empty lines
                if not line.strip():
                    translated.append('')
                    continue
                
                # Skip URLs, code snippets, technical terms
                if self._should_skip(line):
                    translated.append(line)
                    continue
                
                # Translate text
                trans = await asyncio.to_thread(self._translate, line)
                translated.append(trans)
            
            result = '\n'.join(translated)
            logger.info(f"✅ Done: {len(result)} chars")
            
            return TranslationResponse(
                translated_content=result,
                chapter_id=request.chapter_id,
                cached=False
            )
            
        except Exception as e:
            logger.error(f"❌ Error: {e}")
            return TranslationResponse(
                translated_content=request.content,
                chapter_id=request.chapter_id,
                cached=False
            )
    
    def _should_skip(self, line: str) -> bool:
        """Check if line should NOT be translated"""
        skip_patterns = [
            'http://', 'https://', 'www.',
            'sudo ', 'apt ', 'pip ', 'npm ',
            'import ', 'from ', 'def ', 'class ',
            '-->', '|', '    A[', '    B[',  # Mermaid graph syntax
            '#!', '//', '/*',
        ]
        return any(pattern in line for pattern in skip_patterns)
    
    def _translate(self, text: str) -> str:
        """Translate single line"""
        try:
            import torch
            
            text = text.strip()
            if not text or len(text) < 2:
                return text
            
            # Don't translate technical terms
            skip_terms = ['Python', 'ROS', 'API', 'RGB', 'LiDAR', 'IMU', 'AI', 
                         'Gazebo', 'MuJoCo', 'VS Code', 'PyCharm', 'Jupyter']
            if text in skip_terms:
                return text
            
            inputs = self.tokenizer(
                text, 
                return_tensors="pt", 
                truncation=True, 
                max_length=128
            )
            
            with torch.no_grad():
                outputs = self.model.generate(
                    **inputs, 
                    max_length=128, 
                    num_beams=1
                )
            
            return self.tokenizer.decode(outputs[0], skip_special_tokens=True)
            
        except:
            return text

translation_service = TranslationService()