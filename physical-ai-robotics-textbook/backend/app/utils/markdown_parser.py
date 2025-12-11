"""
Markdown parser utility for extracting content from textbook documents.
"""

from typing import List, Dict, Any, Tuple
import re
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
import frontmatter  # For parsing markdown with YAML frontmatter


class MarkdownParser:
    """
    Utility class for parsing markdown documents and extracting structured content.
    Specifically designed for textbook content with modules and chapters.
    """

    def __init__(self):
        pass

    def parse_markdown_file(self, file_path: str) -> Dict[str, Any]:
        """
        Parse a markdown file and extract its content with metadata.

        Args:
            file_path: Path to the markdown file

        Returns:
            Dictionary containing content, metadata, and structure information
        """
        file_path = Path(file_path)

        if not file_path.exists():
            raise FileNotFoundError(f"Markdown file not found: {file_path}")

        # Read the file content
        with open(file_path, "r", encoding="utf-8") as file:
            content = file.read()

        # Parse frontmatter if present
        post = frontmatter.loads(content)
        raw_content = post.content
        metadata = post.metadata

        # Extract title from the first heading if not in frontmatter
        if "title" not in metadata:
            title_match = re.search(r"^#{1,6}\s+(.+)$", raw_content, re.MULTILINE)
            if title_match:
                metadata["title"] = title_match.group(1).strip()
            else:
                metadata["title"] = file_path.stem

        # Extract structure information from file path
        path_parts = file_path.parts
        module = self._extract_module_name(path_parts)
        chapter = self._extract_chapter_name(path_parts, metadata.get("title", ""))

        # Clean and process the content
        cleaned_content = self._clean_markdown_content(raw_content)

        return {
            "content": cleaned_content,
            "raw_content": raw_content,
            "metadata": metadata,
            "module": module,
            "chapter": chapter,
            "file_path": str(file_path),
            "file_name": file_path.name,
            "file_stem": file_path.stem,
        }

    def _extract_module_name(self, path_parts: Tuple[str, ...]) -> str:
        """
        Extract module name from the file path structure.
        Assumes a structure like: docs/module_name/chapter_file.md

        Args:
            path_parts: Tuple of path components

        Returns:
            Extracted module name
        """
        # Look for 'docs' directory and take the next directory as module
        docs_index = -1
        for i, part in enumerate(path_parts):
            if part.lower() in ["docs", "documentation", "textbook"]:
                docs_index = i
                break

        if docs_index != -1 and docs_index + 1 < len(path_parts):
            return path_parts[docs_index + 1]

        # If no docs directory found, use the parent directory name
        return Path(*path_parts[:-1]).name or "unknown_module"

    def _extract_chapter_name(
        self, path_parts: Tuple[str, ...], title: str = ""
    ) -> str:
        """
        Extract chapter name from the file path or title.

        Args:
            path_parts: Tuple of path components
            title: Title from the document

        Returns:
            Extracted chapter name
        """
        file_name = Path(*path_parts).stem if path_parts else "unknown"

        # Use title if available, otherwise use file name
        if title:
            return title
        else:
            # Clean up file name (replace underscores/hyphens with spaces)
            chapter_name = file_name.replace("_", " ").replace("-", " ")
            return chapter_name.title()

    def _clean_markdown_content(self, content: str) -> str:
        """
        Clean markdown content by removing unnecessary elements and normalizing text.

        Args:
            content: Raw markdown content

        Returns:
            Cleaned content string
        """
        # Remove or replace certain markdown elements that might not be useful for embeddings
        # Remove image references
        content = re.sub(r"!\[.*?\]\(.*?\)", "", content)

        # Remove HTML comments
        content = re.sub(r"<!--.*?-->", "", content, flags=re.DOTALL)

        # Normalize whitespace
        content = re.sub(
            r"\n\s*\n", "\n\n", content
        )  # Replace multiple blank lines with single
        content = content.strip()

        return content

    def extract_headings(self, content: str) -> List[Dict[str, Any]]:
        """
        Extract headings from markdown content.

        Args:
            content: Markdown content

        Returns:
            List of dictionaries containing heading information
        """
        headings = []
        lines = content.split("\n")

        for i, line in enumerate(lines):
            # Match markdown headings (h1 to h6)
            heading_match = re.match(r"^(#{1,6})\s+(.+)$", line.strip())
            if heading_match:
                level = len(heading_match.group(1))
                text = heading_match.group(2).strip()

                headings.append(
                    {"level": level, "text": text, "line_number": i, "raw_line": line}
                )

        return headings

    def extract_code_blocks(self, content: str) -> List[Dict[str, Any]]:
        """
        Extract code blocks from markdown content.

        Args:
            content: Markdown content

        Returns:
            List of dictionaries containing code block information
        """
        code_blocks = []

        # Match fenced code blocks
        pattern = r"```(\w*)\n(.*?)```"
        matches = re.finditer(pattern, content, re.DOTALL)

        for match in matches:
            language = match.group(1) if match.group(1) else "text"
            code = match.group(2).strip()

            code_blocks.append(
                {
                    "language": language,
                    "code": code,
                    "start_pos": match.start(),
                    "end_pos": match.end(),
                }
            )

        return code_blocks

    def extract_links(self, content: str) -> List[Dict[str, Any]]:
        """
        Extract links from markdown content.

        Args:
            content: Markdown content

        Returns:
            List of dictionaries containing link information
        """
        links = []

        # Match markdown links [text](url)
        pattern = r"\[([^\]]+)\]\(([^)]+)\)"
        matches = re.finditer(pattern, content)

        for match in matches:
            links.append(
                {
                    "text": match.group(1),
                    "url": match.group(2),
                    "start_pos": match.start(),
                    "end_pos": match.end(),
                }
            )

        return links

    def convert_markdown_to_text(self, markdown_content: str) -> str:
        """
        Convert markdown content to plain text by removing markdown formatting.

        Args:
            markdown_content: Raw markdown content

        Returns:
            Plain text content
        """
        # Convert markdown to HTML first
        html = markdown.markdown(markdown_content)

        # Parse HTML and extract text
        soup = BeautifulSoup(html, "html.parser")
        text = soup.get_text(separator=" ")

        # Clean up extra whitespace
        text = re.sub(r"\s+", " ", text).strip()

        return text

    def parse_multiple_files(self, file_paths: List[str]) -> List[Dict[str, Any]]:
        """
        Parse multiple markdown files and return their content.

        Args:
            file_paths: List of paths to markdown files

        Returns:
            List of dictionaries containing content from each file
        """
        results = []

        for file_path in file_paths:
            try:
                parsed_content = self.parse_markdown_file(file_path)
                results.append(parsed_content)
            except Exception as e:
                print(f"Error parsing file {file_path}: {str(e)}")
                continue

        return results

    def get_file_metadata(self, file_path: str) -> Dict[str, Any]:
        """
        Get basic metadata about a markdown file without parsing its content.

        Args:
            file_path: Path to the markdown file

        Returns:
            Dictionary containing file metadata
        """
        path = Path(file_path)

        if not path.exists():
            raise FileNotFoundError(f"File not found: {file_path}")

        stat = path.stat()

        return {
            "file_path": str(path),
            "file_name": path.name,
            "file_size": stat.st_size,
            "created_time": stat.st_ctime,
            "modified_time": stat.st_mtime,
            "module": self._extract_module_name(path.parts),
            "chapter": self._extract_chapter_name(path.parts),
        }


# Global instance
markdown_parser = MarkdownParser()
