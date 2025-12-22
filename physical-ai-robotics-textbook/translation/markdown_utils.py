import re
from typing import List, Tuple
import markdown
from markdown import Markdown
from markdown.extensions import Extension
from markdown.preprocessors import Preprocessor
from xml.etree.ElementTree import Element
import logging


class CodeBlockPreserver:
    """
    Utility to preserve code blocks during markdown processing
    This ensures code blocks remain unchanged during translation
    """

    def __init__(self):
        self.code_blocks = {}
        self.placeholder_pattern = r'CODE_BLOCK_PLACEHOLDER_\{([0-9a-f-]+)\}'

    def extract_code_blocks(self, markdown_text: str) -> str:
        """
        Extract code blocks from markdown text and replace with placeholders
        Returns text with code blocks replaced by placeholders
        """
        self.code_blocks = {}
        result = markdown_text

        # Pattern to match both inline and fenced code blocks
        code_block_pattern = r'(```[\s\S]*?```|`[^`\n]+`|~~~[\s\S]*?~~~)'

        matches = re.findall(code_block_pattern, result)
        for i, match in enumerate(matches):
            # Generate a unique key for this code block
            key = f"{i:04d}_{hash(match) % (10**8):08d}"
            placeholder = f"CODE_BLOCK_PLACEHOLDER_{{{key}}}"

            # Store the original code block
            self.code_blocks[key] = match

            # Replace the code block with placeholder
            result = result.replace(match, placeholder, 1)

        return result

    def restore_code_blocks(self, translated_text: str) -> str:
        """
        Restore original code blocks in translated text using placeholders
        Returns text with placeholders replaced by original code blocks
        """
        def replace_placeholder(match):
            key = match.group(1)
            return self.code_blocks.get(key, match.group(0))

        result = re.sub(self.placeholder_pattern, replace_placeholder, translated_text)
        return result


def preserve_markdown_structure(markdown_content: str) -> Tuple[str, CodeBlockPreserver]:
    """
    Preserves markdown structure by extracting code blocks before processing
    Returns the processed text and the preserver object to restore code blocks later
    """
    try:
        preserver = CodeBlockPreserver()
        processed_content = preserver.extract_code_blocks(markdown_content)
        return processed_content, preserver
    except Exception as e:
        logging.error(f"Error preserving markdown structure: {str(e)}")
        # Return original content if preservation fails
        return markdown_content, CodeBlockPreserver()


def safely_handle_malformed_markdown(markdown_content: str) -> Tuple[str, CodeBlockPreserver]:
    """
    Safely handle malformed markdown by using fallback preservation methods
    """
    try:
        # Try normal preservation first
        processed_content, preserver = preserve_markdown_structure(markdown_content)
        return processed_content, preserver
    except Exception as e:
        logging.warning(f"Safely handling malformed markdown: {str(e)}")

        # Fallback: return content as-is with empty preserver
        return markdown_content, CodeBlockPreserver()


def restore_markdown_structure(processed_content: str, preserver: CodeBlockPreserver) -> str:
    """
    Restores the original markdown structure by putting back code blocks
    """
    return preserver.restore_code_blocks(processed_content)


def validate_markdown_preservation(original: str, translated: str) -> dict:
    """
    Validates that markdown structure has been preserved during translation
    Returns a dictionary with validation results
    """
    # Extract code blocks from both original and translated
    orig_preserver = CodeBlockPreserver()
    orig_processed = orig_preserver.extract_code_blocks(original)

    trans_preserver = CodeBlockPreserver()
    trans_processed = trans_preserver.extract_code_blocks(translated)

    # Count code blocks
    orig_code_count = len(orig_preserver.code_blocks)
    trans_code_count = len(trans_preserver.code_blocks)

    # Check if all original code blocks are preserved
    code_blocks_preserved = all(
        orig_preserver.code_blocks[key] == trans_preserver.code_blocks.get(key, "")
        for key in orig_preserver.code_blocks.keys()
    )

    # Check for basic markdown elements
    orig_headings = len(re.findall(r'^#{1,6}\s', original, re.MULTILINE))
    trans_headings = len(re.findall(r'^#{1,6}\s', translated, re.MULTILINE))

    orig_lists = len(re.findall(r'^\s*[\*\-\+]\s|^(\d+\.)', original, re.MULTILINE))
    trans_lists = len(re.findall(r'^\s*[\*\-\+]\s|^(\d+\.)', translated, re.MULTILINE))

    return {
        "code_blocks_count_match": orig_code_count == trans_code_count,
        "code_blocks_preserved": code_blocks_preserved,
        "headings_count_match": orig_headings == trans_headings,
        "lists_count_match": orig_lists == trans_lists,
        "original_code_blocks": orig_code_count,
        "translated_code_blocks": trans_code_count
    }


def get_markdown_stats(markdown_content: str) -> dict:
    """
    Get statistics about markdown content structure
    """
    preserver = CodeBlockPreserver()
    processed_content = preserver.extract_code_blocks(markdown_content)

    return {
        "total_chars": len(markdown_content),
        "text_chars": len(processed_content),
        "code_blocks_count": len(preserver.code_blocks),
        "headings_count": len(re.findall(r'^#{1,6}\s', markdown_content, re.MULTILINE)),
        "lists_count": len(re.findall(r'^\s*[\*\-\+]\s|^(\d+\.)', markdown_content, re.MULTILINE)),
        "links_count": len(re.findall(r'\[([^\]]+)\]\(([^)]+)\)', markdown_content)),
        "bold_count": len(re.findall(r'\*\*([^\*]+)\*\*|__([^_]+)__|(?<!\*)\*([^\*]+)\*(?!\*)', markdown_content)),
        "italic_count": len(re.findall(r'(?<!\*)\*([^\*]+)\*(?!\*)|(?<!_)_([^_]+)_(?!_)', markdown_content))
    }