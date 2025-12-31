# RAG Context Control Skill

---
name: rag_context_control
description: Ensure textbook-grounded answers with strict context control and hallucination prevention
version: 1.0.0
project: physical-ai-robotics-textbook
parameters:
  - name: strictness_level
    description: How strict to be about context (strict, moderate, lenient)
    required: false
    default: strict
  - name: min_relevance_score
    description: Minimum similarity score for retrieved chunks (0.0-1.0)
    required: false
    default: 0.7
---

## Purpose

Implement a robust RAG system that:
- **Only answers from textbook content** - Zero hallucination tolerance
- **Explicitly states uncertainty** - Says "I don't know" when context is insufficient
- **Works across all chapters** - Chapter-agnostic implementation
- **Validates retrieved context** - Ensures relevance before answering

## Core Principles

**MUST FOLLOW:**
1. **Grounding Mandate**: Every answer MUST be traceable to retrieved chunks
2. **Honesty Over Guessing**: Admit lack of knowledge rather than hallucinate
3. **Context Transparency**: Show which sources were used
4. **Relevance Filtering**: Reject low-quality retrievals
5. **Chapter Neutrality**: No hardcoded chapter assumptions

## Instructions

### Step 1: Implement Context Retrieval with Quality Control

Generate a retrieval service that filters by relevance:

```python
# app/ai/context_retriever.py

from typing import List, Dict, Optional
from dataclasses import dataclass
from app.config import settings
import logging

logger = logging.getLogger(__name__)

@dataclass
class RetrievedChunk:
    """Represents a retrieved text chunk with metadata"""
    content: str
    chapter: str
    section: str
    relevance_score: float
    chunk_id: str

class ContextRetriever:
    """Retrieves and filters textbook context with quality control"""

    def __init__(self, vector_store, min_relevance: float = 0.7):
        self.vector_store = vector_store
        self.min_relevance = min_relevance

    async def retrieve_context(
        self,
        query: str,
        top_k: int = 5,
        chapter_filter: Optional[str] = None,
        min_score: Optional[float] = None
    ) -> List[RetrievedChunk]:
        """
        Retrieve relevant context from textbook with quality filtering.

        Args:
            query: User's question
            top_k: Maximum number of chunks to retrieve
            chapter_filter: Optional chapter to limit search
            min_score: Override minimum relevance threshold

        Returns:
            List of high-quality retrieved chunks
        """
        # Use provided min_score or default
        threshold = min_score if min_score is not None else self.min_relevance

        # Build metadata filter
        filters = {}
        if chapter_filter:
            filters["chapter"] = chapter_filter

        # Retrieve from vector store
        raw_results = await self.vector_store.query(
            query=query,
            top_k=top_k * 2,  # Retrieve extra, then filter
            filters=filters
        )

        # Filter by relevance score
        filtered_chunks = []
        for result in raw_results:
            if result.score >= threshold:
                filtered_chunks.append(
                    RetrievedChunk(
                        content=result.metadata["text"],
                        chapter=result.metadata.get("chapter", "unknown"),
                        section=result.metadata.get("section", "unknown"),
                        relevance_score=result.score,
                        chunk_id=result.id
                    )
                )

        # Limit to top_k after filtering
        filtered_chunks = filtered_chunks[:top_k]

        # Log retrieval quality
        logger.info(
            f"Retrieved {len(filtered_chunks)}/{len(raw_results)} chunks "
            f"above threshold {threshold:.2f} for query: {query[:100]}"
        )

        return filtered_chunks

    def has_sufficient_context(self, chunks: List[RetrievedChunk]) -> bool:
        """
        Determine if retrieved context is sufficient to answer.

        Criteria:
        - At least 1 chunk above threshold
        - Top chunk has high enough score
        """
        if not chunks:
            return False

        # Check if best chunk is relevant enough
        best_score = max(chunk.relevance_score for chunk in chunks)
        return best_score >= self.min_relevance
```

### Step 2: Implement Strict Prompt Engineering

Create prompt templates that enforce grounding:

```python
# app/ai/prompts.py

from typing import List
from app.ai.context_retriever import RetrievedChunk

class PromptTemplates:
    """Prompt templates that prevent hallucination"""

    @staticmethod
    def strict_rag_prompt(query: str, chunks: List[RetrievedChunk]) -> str:
        """
        STRICT mode: Only answer if context explicitly contains the answer.
        """
        # Format context with clear boundaries
        context_blocks = []
        for i, chunk in enumerate(chunks, 1):
            context_blocks.append(
                f"[Source {i} - Chapter: {chunk.chapter}, Section: {chunk.section}]\n"
                f"{chunk.content}\n"
                f"[End Source {i}]"
            )

        context = "\n\n".join(context_blocks)

        return f"""You are a textbook assistant for "Physical AI & Humanoid Robotics". Your ONLY job is to answer questions using the provided textbook excerpts.

CRITICAL RULES:
1. ONLY use information from the [Source] blocks below
2. DO NOT use your general knowledge about robotics, AI, or programming
3. If the sources don't contain enough information to answer, you MUST say: "I don't have enough information in the textbook to answer this question."
4. Cite which source(s) you used (e.g., "According to Source 1...")
5. If sources conflict, acknowledge both perspectives
6. Never make up examples, code, or explanations not in the sources

Textbook Sources:
{context}

Student Question: {query}

Instructions:
- First, check if the sources contain relevant information
- If NO: Respond with "I don't have enough information in the textbook to answer this question. This topic may not be covered in the available chapters."
- If YES: Answer using ONLY the information from the sources above, citing which source(s) you used

Answer:"""

    @staticmethod
    def moderate_rag_prompt(query: str, chunks: List[RetrievedChunk]) -> str:
        """
        MODERATE mode: Answer from context but can synthesize across sources.
        """
        context_blocks = []
        for i, chunk in enumerate(chunks, 1):
            context_blocks.append(f"Source {i} ({chunk.chapter}):\n{chunk.content}")

        context = "\n\n".join(context_blocks)

        return f"""You are a helpful assistant for the "Physical AI & Humanoid Robotics" textbook.

Use the following textbook excerpts to answer the student's question:

{context}

Student Question: {query}

Guidelines:
- Base your answer primarily on the sources above
- You may synthesize information across multiple sources
- If the sources are insufficient, clearly state what's missing
- Cite which sources you used
- Keep explanations clear and educational

Answer:"""

    @staticmethod
    def no_context_response() -> str:
        """Response when no relevant context is found"""
        return (
            "I don't have enough information in the textbook to answer this question. "
            "This topic may not be covered in the available chapters, or you may need to "
            "rephrase your question to match the textbook content."
        )

    @staticmethod
    def insufficient_context_response(query: str, best_score: float) -> str:
        """Response when context relevance is too low"""
        return (
            f"I found some textbook content, but it doesn't seem directly relevant to your question "
            f"(relevance: {best_score:.1%}). Could you rephrase your question or provide more context?"
        )
```

### Step 3: Implement Response Grounding Service

Create a service that enforces grounding:

```python
# app/services/grounded_qa_service.py

from typing import List, Optional
from app.ai.context_retriever import ContextRetriever, RetrievedChunk
from app.ai.prompts import PromptTemplates
from app.ai.llm_client import LLMClient
from app.models.responses import GroundedAnswer, Source
import logging

logger = logging.getLogger(__name__)

class GroundedQAService:
    """Question answering service with strict grounding enforcement"""

    def __init__(
        self,
        retriever: ContextRetriever,
        llm_client: LLMClient,
        strictness: str = "strict"
    ):
        self.retriever = retriever
        self.llm_client = llm_client
        self.strictness = strictness
        self.prompts = PromptTemplates()

    async def answer_question(
        self,
        query: str,
        chapter_filter: Optional[str] = None,
        top_k: int = 5
    ) -> GroundedAnswer:
        """
        Answer a question with strict grounding in textbook content.

        Returns:
            GroundedAnswer with response and source attribution
        """
        # Step 1: Retrieve context
        chunks = await self.retriever.retrieve_context(
            query=query,
            top_k=top_k,
            chapter_filter=chapter_filter
        )

        # Step 2: Check context sufficiency
        if not chunks:
            logger.warning(f"No context found for query: {query}")
            return GroundedAnswer(
                answer=self.prompts.no_context_response(),
                sources=[],
                grounded=False,
                confidence="none"
            )

        if not self.retriever.has_sufficient_context(chunks):
            best_score = max(chunk.relevance_score for chunk in chunks)
            logger.warning(
                f"Insufficient context quality for query: {query} "
                f"(best score: {best_score:.2f})"
            )
            return GroundedAnswer(
                answer=self.prompts.insufficient_context_response(query, best_score),
                sources=self._format_sources(chunks),
                grounded=False,
                confidence="low"
            )

        # Step 3: Generate grounded response
        prompt = self._build_prompt(query, chunks)

        # Add system message for extra grounding
        system_message = (
            "You are a textbook assistant. NEVER use knowledge outside the provided context. "
            "If you cannot answer from the context, explicitly say so."
        )

        answer = await self.llm_client.generate(
            prompt=prompt,
            system_message=system_message,
            temperature=0.3,  # Lower temperature for more conservative responses
            max_tokens=1000
        )

        # Step 4: Validate response is grounded
        is_grounded = self._validate_grounding(answer, chunks)
        confidence = self._calculate_confidence(chunks, is_grounded)

        # Step 5: Return grounded answer
        return GroundedAnswer(
            answer=answer,
            sources=self._format_sources(chunks),
            grounded=is_grounded,
            confidence=confidence,
            retrieved_chunks_count=len(chunks)
        )

    def _build_prompt(self, query: str, chunks: List[RetrievedChunk]) -> str:
        """Build prompt based on strictness level"""
        if self.strictness == "strict":
            return self.prompts.strict_rag_prompt(query, chunks)
        elif self.strictness == "moderate":
            return self.prompts.moderate_rag_prompt(query, chunks)
        else:
            return self.prompts.strict_rag_prompt(query, chunks)  # Default to strict

    def _validate_grounding(self, answer: str, chunks: List[RetrievedChunk]) -> bool:
        """
        Validate that the answer is grounded in the retrieved context.

        Simple heuristic: Check for explicit uncertainty statements.
        More sophisticated: Use NLI model or fact-checking.
        """
        # Check for explicit "I don't know" statements
        uncertainty_phrases = [
            "don't have enough information",
            "not covered in the textbook",
            "cannot answer",
            "insufficient information",
            "not found in the sources"
        ]

        answer_lower = answer.lower()

        # If answer contains uncertainty, mark as not grounded (honest uncertainty)
        if any(phrase in answer_lower for phrase in uncertainty_phrases):
            return False

        # If answer references sources, likely grounded
        if "source" in answer_lower or "according to" in answer_lower:
            return True

        # Default: assume grounded if we got here
        # (more sophisticated validation could use NLI)
        return True

    def _calculate_confidence(
        self,
        chunks: List[RetrievedChunk],
        is_grounded: bool
    ) -> str:
        """Calculate confidence level based on context quality"""
        if not is_grounded:
            return "low"

        if not chunks:
            return "none"

        avg_score = sum(c.relevance_score for c in chunks) / len(chunks)

        if avg_score >= 0.85:
            return "high"
        elif avg_score >= 0.70:
            return "medium"
        else:
            return "low"

    def _format_sources(self, chunks: List[RetrievedChunk]) -> List[Source]:
        """Convert chunks to Source objects"""
        return [
            Source(
                chapter=chunk.chapter,
                section=chunk.section,
                content_snippet=chunk.content[:200] + "...",
                relevance_score=chunk.relevance_score
            )
            for chunk in chunks
        ]
```

### Step 4: Define Response Models

```python
# app/models/responses.py (additions)

from pydantic import BaseModel, Field
from typing import List, Literal

class Source(BaseModel):
    """Source attribution for grounded answers"""
    chapter: str
    section: str
    content_snippet: str
    relevance_score: float = Field(..., ge=0.0, le=1.0)

class GroundedAnswer(BaseModel):
    """Response model for grounded Q&A"""
    answer: str = Field(..., description="The answer text")
    sources: List[Source] = Field(default_factory=list)
    grounded: bool = Field(..., description="Whether answer is grounded in context")
    confidence: Literal["none", "low", "medium", "high"]
    retrieved_chunks_count: int = Field(0, description="Number of chunks retrieved")

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "According to Source 1, inverse kinematics...",
                "sources": [
                    {
                        "chapter": "03-kinematics",
                        "section": "inverse-kinematics",
                        "content_snippet": "Inverse kinematics is the process of...",
                        "relevance_score": 0.92
                    }
                ],
                "grounded": True,
                "confidence": "high",
                "retrieved_chunks_count": 3
            }
        }
```

### Step 5: Create API Route

```python
# app/routes/grounded_qa.py

from fastapi import APIRouter, HTTPException, Depends, Query
from app.models.requests import QuestionRequest
from app.models.responses import GroundedAnswer
from app.services.grounded_qa_service import GroundedQAService
from app.dependencies import get_grounded_qa_service
from typing import Optional

router = APIRouter()

@router.post("/ask", response_model=GroundedAnswer)
async def ask_grounded_question(
    request: QuestionRequest,
    chapter: Optional[str] = Query(None, description="Filter by chapter (e.g., '01-ros2')"),
    strictness: str = Query("strict", regex="^(strict|moderate|lenient)$"),
    service: GroundedQAService = Depends(get_grounded_qa_service)
):
    """
    Ask a question with strict grounding in textbook content.

    **Grounding Levels:**
    - `strict`: Only answers from explicit textbook content
    - `moderate`: Can synthesize across sources
    - `lenient`: More flexible interpretation

    **Response:**
    - `grounded=true`: Answer is based on textbook content
    - `grounded=false`: Insufficient context (honest uncertainty)
    - `confidence`: Quality assessment (none/low/medium/high)
    """
    try:
        # Update service strictness
        service.strictness = strictness

        # Process question
        answer = await service.answer_question(
            query=request.question,
            chapter_filter=chapter,
            top_k=5
        )

        return answer

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

### Step 6: Testing Strategy

Create tests that verify grounding behavior:

```python
# tests/test_grounding.py

import pytest
from app.services.grounded_qa_service import GroundedQAService
from app.ai.context_retriever import RetrievedChunk

class TestGroundingBehavior:
    """Test that the system enforces grounding"""

    @pytest.mark.asyncio
    async def test_no_context_returns_uncertainty(self, qa_service):
        """When no context is found, should admit uncertainty"""
        # Mock retriever to return empty list
        qa_service.retriever.retrieve_context = lambda *args, **kwargs: []

        result = await qa_service.answer_question("What is quantum computing?")

        assert result.grounded == False
        assert result.confidence == "none"
        assert "don't have enough information" in result.answer.lower()

    @pytest.mark.asyncio
    async def test_low_relevance_context_rejected(self, qa_service):
        """When context relevance is too low, should reject it"""
        # Mock low-relevance chunks
        low_quality_chunks = [
            RetrievedChunk(
                content="Random unrelated text",
                chapter="01-ros2",
                section="intro",
                relevance_score=0.3,  # Below threshold
                chunk_id="test-1"
            )
        ]
        qa_service.retriever.retrieve_context = lambda *args, **kwargs: low_quality_chunks

        result = await qa_service.answer_question("What is inverse kinematics?")

        assert result.grounded == False
        assert result.confidence == "low"

    @pytest.mark.asyncio
    async def test_hallucination_prevention(self, qa_service):
        """Should not answer questions about topics not in textbook"""
        # Query about topic definitely not in robotics textbook
        result = await qa_service.answer_question(
            "How do I bake a chocolate cake?"
        )

        # Should admit it doesn't know, not hallucinate an answer
        assert result.grounded == False or "don't have" in result.answer.lower()

    @pytest.mark.asyncio
    async def test_grounded_answer_cites_sources(self, qa_service):
        """Grounded answers should cite sources"""
        # Mock high-quality chunks
        good_chunks = [
            RetrievedChunk(
                content="Inverse kinematics calculates joint angles from end-effector position.",
                chapter="03-kinematics",
                section="inverse-kinematics",
                relevance_score=0.95,
                chunk_id="test-1"
            )
        ]
        qa_service.retriever.retrieve_context = lambda *args, **kwargs: good_chunks

        result = await qa_service.answer_question("What is inverse kinematics?")

        assert result.grounded == True
        assert result.confidence in ["medium", "high"]
        assert len(result.sources) > 0
        # Should reference sources
        assert "source" in result.answer.lower() or "according to" in result.answer.lower()

    @pytest.mark.asyncio
    async def test_chapter_agnostic(self, qa_service):
        """Should work across all chapters without hardcoding"""
        chapters = ["01-ros2", "02-simulation", "03-kinematics", "04-vla"]

        for chapter in chapters:
            # Should not fail for any chapter
            result = await qa_service.answer_question(
                query="Explain the main concepts",
                chapter_filter=chapter
            )

            # Should either return grounded answer or honest uncertainty
            assert isinstance(result.grounded, bool)
            assert result.confidence in ["none", "low", "medium", "high"]
```

### Step 7: Configuration

```python
# app/config.py (additions)

class Settings(BaseSettings):
    # ... existing settings ...

    # RAG Grounding Configuration
    MIN_RELEVANCE_SCORE: float = 0.7
    STRICTNESS_LEVEL: str = "strict"  # strict, moderate, lenient
    MAX_CONTEXT_CHUNKS: int = 5

    # Hallucination Prevention
    ENABLE_GROUNDING_VALIDATION: bool = True
    REQUIRE_SOURCE_CITATION: bool = True
    LOW_CONFIDENCE_THRESHOLD: float = 0.6
```

### Step 8: Monitoring & Logging

```python
# app/utils/grounding_monitor.py

import logging
from typing import Dict
from collections import defaultdict

logger = logging.getLogger(__name__)

class GroundingMonitor:
    """Monitor grounding quality and detect hallucination patterns"""

    def __init__(self):
        self.stats = defaultdict(int)

    def log_response(
        self,
        query: str,
        grounded: bool,
        confidence: str,
        chunks_count: int
    ):
        """Log each response for monitoring"""

        self.stats["total_queries"] += 1

        if grounded:
            self.stats["grounded_responses"] += 1
        else:
            self.stats["ungrounded_responses"] += 1
            logger.warning(f"Ungrounded response for query: {query[:100]}")

        self.stats[f"confidence_{confidence}"] += 1

        if chunks_count == 0:
            self.stats["no_context_found"] += 1
            logger.info(f"No context found for: {query[:100]}")

    def get_metrics(self) -> Dict:
        """Get grounding metrics"""
        total = self.stats["total_queries"]
        if total == 0:
            return {}

        return {
            "total_queries": total,
            "grounding_rate": self.stats["grounded_responses"] / total,
            "uncertainty_rate": self.stats["ungrounded_responses"] / total,
            "no_context_rate": self.stats["no_context_found"] / total,
            "confidence_distribution": {
                "high": self.stats["confidence_high"] / total,
                "medium": self.stats["confidence_medium"] / total,
                "low": self.stats["confidence_low"] / total,
                "none": self.stats["confidence_none"] / total
            }
        }
```

## Validation Checklist

After implementation, verify:

- [ ] **Retrieval Quality**: Only chunks above threshold are used
- [ ] **Honest Uncertainty**: System says "I don't know" when appropriate
- [ ] **Source Citation**: Grounded answers cite specific sources
- [ ] **Chapter Agnostic**: No hardcoded chapter IDs or assumptions
- [ ] **Hallucination Prevention**: Cannot answer off-topic questions
- [ ] **Backend Only**: No frontend logic mixed in
- [ ] **Configurable Strictness**: Can adjust grounding level
- [ ] **Monitoring**: Logs grounding quality metrics
- [ ] **Testing**: Tests cover edge cases and failure modes
- [ ] **Documentation**: Clear API docs and usage examples

## Anti-Patterns to Avoid

❌ **Don't**: Allow LLM to use general knowledge
✅ **Do**: Explicit system messages forbidding external knowledge

❌ **Don't**: Accept low-relevance retrievals
✅ **Do**: Filter by minimum relevance threshold

❌ **Don't**: Silently return uncertain answers as confident
✅ **Do**: Include confidence scores and grounding flags

❌ **Don't**: Hardcode chapter-specific logic
✅ **Do**: Use metadata filters dynamically

❌ **Don't**: Mix frontend rendering logic
✅ **Do**: Return pure JSON with grounding metadata

## Example Usage

```python
# Example 1: Strict grounding (default)
POST /api/grounded-qa/ask
{
  "question": "What is inverse kinematics?",
  "strictness": "strict"
}

Response:
{
  "answer": "According to Source 1 (Chapter 3, Section: Inverse Kinematics), inverse kinematics is the process of calculating joint angles required to position a robot's end-effector at a desired location...",
  "sources": [
    {
      "chapter": "03-kinematics",
      "section": "inverse-kinematics",
      "content_snippet": "Inverse kinematics calculates joint angles...",
      "relevance_score": 0.94
    }
  ],
  "grounded": true,
  "confidence": "high",
  "retrieved_chunks_count": 3
}

# Example 2: Insufficient context
POST /api/grounded-qa/ask
{
  "question": "How do I train a quantum neural network?"
}

Response:
{
  "answer": "I don't have enough information in the textbook to answer this question. This topic may not be covered in the available chapters.",
  "sources": [],
  "grounded": false,
  "confidence": "none",
  "retrieved_chunks_count": 0
}
```

## Output

When this skill is invoked, it should:

1. Generate all service files for grounding control
2. Create prompt templates that enforce grounding
3. Implement quality filtering for retrieved context
4. Add monitoring/logging for grounding metrics
5. Create comprehensive tests for edge cases
6. Document the grounding strategy in README

## Success Criteria

- ✅ System never hallucinates answers outside textbook
- ✅ Explicitly states "I don't know" when context is insufficient
- ✅ All answers cite specific textbook sources
- ✅ Works identically across all chapters (no hardcoding)
- ✅ Backend service with no frontend concerns
- ✅ Configurable strictness levels
- ✅ Comprehensive test coverage
