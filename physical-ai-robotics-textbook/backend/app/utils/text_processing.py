"""
Text processing utilities for document chunking and preprocessing.
"""

from typing import List, Dict, Any, Tuple
from app.config import settings
from app.utils.logger import get_logger
import re


class TextProcessor:
    """
    Utility class for text processing, including chunking, cleaning, and preprocessing.
    """

    def __init__(self):
        self.chunk_size = settings.chunk_size
        self.chunk_overlap = settings.chunk_overlap
        self.logger = get_logger("text_processor")

    def chunk_text(
        self, text: str, chunk_size: int = None, overlap: int = None
    ) -> List[Dict[str, Any]]:
        """
        Split text into chunks of specified size with overlap.

        Args:
            text: Input text to be chunked
            chunk_size: Size of each chunk (defaults to config)
            overlap: Overlap between chunks (defaults to config)

        Returns:
            List of dictionaries containing chunk content and metadata
        """
        if chunk_size is None:
            chunk_size = self.chunk_size
        if overlap is None:
            overlap = self.chunk_overlap

        if not text:
            return []

        chunks = []
        start = 0
        chunk_index = 0

        while start < len(text):
            # Calculate end position
            end = start + chunk_size

            # If this is the last chunk, make sure we include all remaining text
            if end >= len(text):
                end = len(text)
            else:
                # Try to break at sentence boundary if possible
                # Look for sentence endings near the end of the chunk
                search_start = max(
                    start + chunk_size // 2, start
                )  # Start searching from halfway through
                sentence_end = -1

                # Look for sentence boundaries (. ! ? followed by space or end of text)
                for i in range(min(search_start, len(text)), min(end + 20, len(text))):
                    if text[i] in ".!?" and (
                        i + 1 >= len(text) or text[i + 1].isspace()
                    ):
                        sentence_end = i + 1
                        break

                # If we found a sentence boundary, use it; otherwise, use the original end
                if sentence_end != -1 and sentence_end > start + chunk_size // 2:
                    end = sentence_end
                else:
                    # If no sentence boundary found, try to break at word boundary
                    for i in range(
                        min(end, len(text)) - 1, max(start, start + chunk_size // 2), -1
                    ):
                        if text[i].isspace():
                            end = i
                            break

            # Extract the chunk
            chunk_text = text[start:end]

            # Add chunk to list
            chunks.append(
                {
                    "content": chunk_text,
                    "start_pos": start,
                    "end_pos": end,
                    "chunk_index": chunk_index,
                    "length": len(chunk_text),
                }
            )

            # Move start position
            start = end - overlap if overlap < end else end
            chunk_index += 1

            # Prevent infinite loop
            if start >= len(text):
                break

        self.logger.info(f"Text chunked into {len(chunks)} chunks")
        return chunks

    def chunk_by_sentences(
        self, text: str, max_chunk_size: int = None
    ) -> List[Dict[str, Any]]:
        """
        Split text into sentence-based chunks.

        Args:
            text: Input text to be chunked
            max_chunk_size: Maximum size of each chunk

        Returns:
            List of dictionaries containing sentence-based chunks
        """
        if max_chunk_size is None:
            max_chunk_size = self.chunk_size

        # Split text into sentences
        sentence_pattern = r"(?<=[.!?])\s+"
        sentences = re.split(sentence_pattern, text)

        chunks = []
        current_chunk = ""
        current_start_pos = 0
        chunk_index = 0

        for i, sentence in enumerate(sentences):
            # Check if adding this sentence would exceed the chunk size
            if len(current_chunk) + len(sentence) > max_chunk_size and current_chunk:
                # Save the current chunk
                chunks.append(
                    {
                        "content": current_chunk.strip(),
                        "start_pos": current_start_pos,
                        "end_pos": current_start_pos + len(current_chunk),
                        "chunk_index": chunk_index,
                        "length": len(current_chunk),
                    }
                )

                # Start a new chunk with the current sentence
                current_chunk = sentence
                current_start_pos = text.find(sentence, current_start_pos)
                chunk_index += 1
            else:
                # Add sentence to current chunk
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append(
                {
                    "content": current_chunk.strip(),
                    "start_pos": current_start_pos,
                    "end_pos": current_start_pos + len(current_chunk),
                    "chunk_index": chunk_index,
                    "length": len(current_chunk),
                }
            )

        self.logger.info(f"Text chunked into {len(chunks)} sentence-based chunks")
        return chunks

    def chunk_by_paragraphs(self, text: str) -> List[Dict[str, Any]]:
        """
        Split text into paragraph-based chunks.

        Args:
            text: Input text to be chunked

        Returns:
            List of dictionaries containing paragraph-based chunks
        """
        # Split text by double newlines (paragraphs)
        paragraphs = [p.strip() for p in text.split("\n\n") if p.strip()]

        chunks = []
        current_pos = 0

        for i, paragraph in enumerate(paragraphs):
            chunks.append(
                {
                    "content": paragraph,
                    "start_pos": current_pos,
                    "end_pos": current_pos + len(paragraph),
                    "chunk_index": i,
                    "length": len(paragraph),
                }
            )
            current_pos += len(paragraph) + 2  # +2 for the \n\n

        self.logger.info(f"Text chunked into {len(chunks)} paragraph-based chunks")
        return chunks

    def clean_text(self, text: str) -> str:
        """
        Clean text by removing unnecessary elements and normalizing.

        Args:
            text: Input text to clean

        Returns:
            Cleaned text
        """
        # Remove extra whitespace
        text = re.sub(r"\s+", " ", text)

        # Remove special characters but keep essential punctuation
        # This regex keeps letters, numbers, basic punctuation, and whitespace
        text = re.sub(r'[^\w\s\.\!\?,;:\-\'"()\[\]{}]', " ", text)

        # Normalize whitespace again after special character removal
        text = re.sub(r"\s+", " ", text).strip()

        return text

    def preprocess_for_embedding(self, text: str) -> str:
        """
        Preprocess text specifically for embedding generation.

        Args:
            text: Input text to preprocess

        Returns:
            Preprocessed text optimized for embedding
        """
        # Clean the text first
        text = self.clean_text(text)

        # Remove very short chunks that might not be meaningful
        if len(text) < 10:
            return ""

        # Normalize the text
        text = text.strip()

        return text

    def merge_chunks(
        self, chunks: List[Dict[str, Any]], max_size: int = None
    ) -> List[Dict[str, Any]]:
        """
        Merge small chunks to approach the target size.

        Args:
            chunks: List of chunk dictionaries
            max_size: Maximum size for merged chunks

        Returns:
            List of merged chunks
        """
        if max_size is None:
            max_size = self.chunk_size

        if not chunks:
            return []

        merged_chunks = []
        current_chunk = {
            "content": "",
            "start_pos": chunks[0]["start_pos"],
            "end_pos": chunks[0]["end_pos"],
            "chunk_index": 0,
            "length": 0,
        }

        for chunk in chunks:
            # Check if adding this chunk would exceed the max size
            new_content = current_chunk["content"] + " " + chunk["content"]
            if len(new_content) <= max_size or len(current_chunk["content"]) == 0:
                # Add to current chunk
                if current_chunk["content"]:
                    current_chunk["content"] = new_content
                    current_chunk["end_pos"] = chunk["end_pos"]
                    current_chunk["length"] = len(current_chunk["content"])
                else:
                    # First chunk
                    current_chunk["content"] = chunk["content"]
                    current_chunk["start_pos"] = chunk["start_pos"]
                    current_chunk["end_pos"] = chunk["end_pos"]
                    current_chunk["length"] = len(current_chunk["content"])
            else:
                # Save current chunk and start new one
                merged_chunks.append(current_chunk)
                current_chunk = {
                    "content": chunk["content"],
                    "start_pos": chunk["start_pos"],
                    "end_pos": chunk["end_pos"],
                    "chunk_index": len(merged_chunks),
                    "length": len(chunk["content"]),
                }

        # Add the last chunk if it has content
        if current_chunk["content"]:
            merged_chunks.append(current_chunk)

        self.logger.info(
            f"Merged {len(chunks)} chunks into {len(merged_chunks)} larger chunks"
        )
        return merged_chunks

    def get_chunk_statistics(self, chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Get statistics about the chunks.

        Args:
            chunks: List of chunk dictionaries

        Returns:
            Dictionary containing statistics
        """
        if not chunks:
            return {
                "total_chunks": 0,
                "total_characters": 0,
                "avg_chunk_size": 0,
                "min_chunk_size": 0,
                "max_chunk_size": 0,
            }

        lengths = [chunk["length"] for chunk in chunks]
        total_chars = sum(lengths)

        return {
            "total_chunks": len(chunks),
            "total_characters": total_chars,
            "avg_chunk_size": total_chars / len(chunks),
            "min_chunk_size": min(lengths),
            "max_chunk_size": max(lengths),
            "size_variance": sum(
                (x - (total_chars / len(chunks))) ** 2 for x in lengths
            )
            / len(chunks),
        }

    def validate_chunk_size(
        self, chunk: str, min_size: int = 10, max_size: int = None
    ) -> bool:
        """
        Validate that a chunk is within acceptable size limits.

        Args:
            chunk: Text chunk to validate
            min_size: Minimum acceptable size
            max_size: Maximum acceptable size (defaults to config)

        Returns:
            True if chunk size is acceptable, False otherwise
        """
        if max_size is None:
            max_size = self.chunk_size * 2  # Allow some flexibility

        length = len(chunk)
        return min_size <= length <= max_size


# Global instance
text_processor = TextProcessor()
