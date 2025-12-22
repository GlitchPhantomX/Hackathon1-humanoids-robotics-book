import re
from typing import List, Tuple


class MarkdownChunker:
    """
    Utility for splitting markdown content into sections based on headers
    """

    @staticmethod
    def split_by_headers(content: str) -> List[str]:
        """
        Split markdown content into sections based on headers (h1, h2, h3, etc.)

        Args:
            content: The markdown content to split

        Returns:
            List of content sections
        """
        # Split content by markdown headers (#, ##, ###, etc.)
        header_pattern = r'^(#{1,6})\s+(.*?)\n'
        sections = []

        # Split the content by headers, keeping the split points
        parts = re.split(header_pattern, content, flags=re.MULTILINE)

        # Reconstruct sections with headers
        current_section = ""

        # If the first part doesn't start with a header, it's content before any header
        if parts and not parts[0].startswith('#'):
            current_section = parts.pop(0).strip()

        i = 0
        while i < len(parts):
            if i + 2 <= len(parts):  # Ensure we have header level, header text, and content
                header_level = parts[i]  # e.g., "##"
                header_text = parts[i + 1]  # e.g., "Introduction"

                # The content after this header (before the next header)
                if i + 2 < len(parts):
                    header_content = parts[i + 2]
                else:
                    header_content = ""

                # Add the current accumulated section to the list if it's not empty
                if current_section:
                    sections.append(current_section.strip())

                # Create a new section with the header and its content
                current_section = f"{header_level} {header_text}\n{header_content}"
                i += 3
            else:
                # Handle remaining content if any
                if parts[i].strip():
                    current_section += parts[i]
                i += 1

        # Add the last section if it exists
        if current_section.strip():
            sections.append(current_section.strip())

        # Filter out any empty sections
        sections = [section for section in sections if section.strip()]

        return sections

    @staticmethod
    def split_by_paragraphs(content: str, max_chars: int = 1000) -> List[str]:
        """
        Split content by paragraphs ensuring each chunk doesn't exceed max_chars

        Args:
            content: The content to split
            max_chars: Maximum characters per chunk

        Returns:
            List of content chunks
        """
        paragraphs = content.split('\n\n')
        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            # Check if adding this paragraph would exceed the limit
            if len(current_chunk) + len(paragraph) <= max_chars:
                current_chunk += paragraph + '\n\n'
            else:
                # If current chunk is not empty, save it and start a new one
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                current_chunk = paragraph + '\n\n'

        # Add the last chunk if it exists
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    @staticmethod
    def split_by_custom_delimiter(content: str, delimiter: str = "<!-- SECTION -->") -> List[str]:
        """
        Split content by a custom delimiter

        Args:
            content: The content to split
            delimiter: The delimiter to split by

        Returns:
            List of content sections
        """
        sections = content.split(delimiter)
        return [section.strip() for section in sections if section.strip()]

    @staticmethod
    def chunk_content(content: str, method: str = "headers", **kwargs) -> List[str]:
        """
        Main method to chunk content using specified method

        Args:
            content: The content to chunk
            method: The method to use for chunking ("headers", "paragraphs", "custom")
            **kwargs: Additional arguments for specific methods

        Returns:
            List of content chunks
        """
        if method == "headers":
            return MarkdownChunker.split_by_headers(content)
        elif method == "paragraphs":
            max_chars = kwargs.get('max_chars', 1000)
            return MarkdownChunker.split_by_paragraphs(content, max_chars)
        elif method == "custom":
            delimiter = kwargs.get('delimiter', "<!-- SECTION -->")
            return MarkdownChunker.split_by_custom_delimiter(content, delimiter)
        else:
            # Default to splitting by headers
            return MarkdownChunker.split_by_headers(content)


# Example usage:
if __name__ == "__main__":
    sample_content = """
# Introduction

This is the introduction section with some content.

## Background

Here is some background information that provides context.

### Technical Details

These are the technical details about the topic.

# Main Concepts

This section covers the main concepts.

## Implementation

How to implement the concepts in practice.
"""

    chunker = MarkdownChunker()
    sections = chunker.split_by_headers(sample_content)
    for i, section in enumerate(sections):
        print(f"Section {i + 1}:")
        print(section)
        print("-" * 40)