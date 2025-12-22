import logging
import time
from typing import List
import os
from openai import AsyncOpenAI
from models.personalization import UserProfile

logger = logging.getLogger(__name__)


class AIContentPersonalizationService:
    """
    Service for AI-based content personalization using OpenAI GPT-4
    """
    
    def __init__(self):
        self.api_key = os.getenv("OPENROUTER_API_KEY")
        self.logger = logging.getLogger(__name__)
        
        if not self.api_key:
            self.logger.error("❌ OPENROUTER_API_KEY not found in environment!")
            self.client = None
        else:
            self.client = AsyncOpenAI(api_key=self.api_key)
            self.logger.info("✅ AI Personalization Service initialized with OpenAI")
            self.logger.info(f"🔑 API Key present: {self.api_key[:20]}...")
    
    async def personalize_content(
        self,
        chapter_content: str,
        user_profile: UserProfile
    ) -> str:
        """
        Personalize the chapter content based on user profile using OpenAI
        """
        
        if not self.client:
            self.logger.warning("⚠️ No OpenAI client - returning original content")
            return chapter_content
        
        try:
            # Create personalization prompt
            prompt = self._create_personalization_prompt(chapter_content, user_profile)
            
            self.logger.info(f"🤖 Calling OpenAI API...")
            self.logger.info(f"📝 Content length: {len(chapter_content)} chars")
            start_time = time.time()
            
            # Call OpenAI API with error handling
            response = await self.client.chat.completions.create(
                model="mistralai/devstral-2512:free",
                messages=[
                    {
                        "role": "system",
                        "content": "You are an expert technical writer specializing in personalized robotics education. You MUST preserve ALL HTML/JSX structure and Docusaurus components while adapting ONLY the text content to match students' backgrounds."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.7,
                max_tokens=8000,  # ✅ Increased for longer content
                timeout=30.0  # ✅ Increased timeout
            )
            
            personalized_content = response.choices[0].message.content
            
            processing_time = (time.time() - start_time) * 1000
            self.logger.info(f"✅ OpenAI responded in {processing_time:.0f}ms")
            self.logger.info(f"📊 Original: {len(chapter_content)} → Personalized: {len(personalized_content)} chars")
            
            return personalized_content
            
        except Exception as e:
            self.logger.error(f"❌ OpenAI API Error: {type(e).__name__}: {str(e)}")
            self.logger.exception("Full traceback:")
            return chapter_content
    
    def _create_personalization_prompt(
        self, 
        chapter_content: str, 
        user_profile: UserProfile
    ) -> str:
        """
        Create a structured prompt for personalizing content based on user profile
        """
        
        level_desc = {
            "none": "absolute beginner with NO prior robotics experience",
            "beginner": "beginner with basic programming knowledge",
            "intermediate": "intermediate learner with some robotics projects",
            "advanced": "advanced learner with professional experience",
            "expert": "expert with research or industry background"
        }
        
        level = level_desc.get(user_profile.roboticsExperience.lower(), "intermediate learner")
        
        prompt = f"""You are personalizing a Docusaurus/MDX textbook chapter for a {level}.

**Student Profile:**
- Experience: {user_profile.roboticsExperience}
- Software: {user_profile.softwareBackground}
- Hardware: {user_profile.hardwareBackground}
- Languages: {user_profile.programmingLanguages}
- AI/ML: {user_profile.aiMlExperience}
- ROS: {'Yes' if user_profile.hasRosExperience else 'No'}
- GPU: {'Yes' if user_profile.hasGpuAccess else 'No'}
- Goals: {user_profile.learningGoals}

**CRITICAL REQUIREMENTS - MUST FOLLOW EXACTLY:**

1. **PRESERVE ALL HTML/JSX STRUCTURE:**
   - Keep ALL `<h1>`, `<h2>`, `<h3>` tags with their className attributes
   - Keep ALL `<div className="second-heading">`, `<div className="underline-class">`, `<div className="border-line">`
   - Keep ALL `<div className="full-content">` and `<div className="summary-content">`
   - Keep ALL import statements at the top
   - Keep ALL frontmatter (between --- lines)
   - Keep ALL React components like `<ReadingTime>`, `<PersonalizeButton>`
   - Keep ALL `:::tip`, `:::caution` admonitions
   - Keep ALL markdown tables with | symbols
   - Keep ALL code blocks with ``` markers
   - DO NOT convert HTML tags to markdown equivalents

2. **ONLY CHANGE TEXT CONTENT:**
   - Modify ONLY the text BETWEEN tags
   - Add beginner tips as plain text inline
   - Add analogies within existing sentences
   - DO NOT add new HTML/JSX tags
   - DO NOT remove existing HTML/JSX tags
   - DO NOT change any className attributes
   - DO NOT modify component props

3. **PERSONALIZATION RULES:**

FOR BEGINNERS (none/beginner):
- Add simple analogies inline: "Physical AI (think of it as a smart robot friend that can see and touch things)"
- Add 💡 emoji tips within bullet points: "- Sensors 💡 (these are like the robot's eyes and ears)"
- Use encouraging language: "Don't worry, we'll explain this step by step!"
- Simplify terms in parentheses: "actuators (the robot's muscles)"
- Break long sentences into shorter ones
- Add reassuring phrases: "This might seem complex, but..."

FOR INTERMEDIATE:
- Add practical insights related to {user_profile.programmingLanguages}
- Reference their goals: {user_profile.learningGoals}
- Add "Pro tip:" notes inline
- Connect concepts to real projects

FOR ADVANCED:
- Use technical terminology without explanation
- Add optimization notes: "Note: For better performance..."
- Reference research papers or industry practices
- Add advanced considerations

4. **STRUCTURE PRESERVATION EXAMPLES:**

BEFORE:
```
<h2 className="second-heading">
 What is Physical AI?
</h2>
<div className="underline-class"></div>

Physical AI represents the convergence of AI and physical systems
```

AFTER (Beginner):
```
<h2 className="second-heading">
 What is Physical AI?
</h2>
<div className="underline-class"></div>

Physical AI represents the convergence of AI and physical systems (imagine teaching a computer to control a robot that can touch, see, and move in the real world! 💡)
```

BEFORE:
```
**Module**: 00 - Introduction
**Learning Objectives**:
- • Understand the fundamental concepts
```

AFTER (Beginner):
```
**Module**: 00 - Introduction
**Learning Objectives**:
- • Understand the fundamental concepts 💡 (don't worry, we'll start from the very basics!)
```

5. **WHAT TO KEEP EXACTLY AS-IS:**
   - All `---` frontmatter sections
   - All `import` statements
   - All `<ReadingTime>` components
   - All `<PersonalizeButton>` components
   - All `:::tip`, `:::caution`, `:::note` blocks (only modify text inside)
   - All code blocks ```python (only add comments if helpful)
   - All markdown tables
   - All heading hierarchy (don't change ### to ##)
   - All className attributes
   - All `<div>` tags

**Original Content:**
{chapter_content}

**IMPORTANT:** Return the COMPLETE personalized MDX file with ALL HTML/JSX structure, imports, frontmatter, and components EXACTLY preserved. Only the TEXT content between tags should be personalized.

**Personalized Content:**"""
        
        return prompt
    
    async def personalize_content_sections(
        self,
        sections: List[str],
        user_profile: UserProfile
    ) -> List[str]:
        """Personalize multiple content sections"""
        self.logger.info(f"📚 Personalizing {len(sections)} sections...")
        personalized_sections = []
        
        for i, section in enumerate(sections, 1):
            self.logger.info(f"  📄 Section {i}/{len(sections)}...")
            personalized = await self.personalize_content(section, user_profile)
            personalized_sections.append(personalized)
        
        return personalized_sections


# Singleton instance
ai_personalization_service = AIContentPersonalizationService()