"""
Test suite for markdown structure preservation during translation
"""
import pytest
from .markdown_utils import (
    preserve_markdown_structure,
    restore_markdown_structure,
    validate_markdown_preservation
)


class TestMarkdownPreservation:
    """
    Tests to verify that markdown structure is preserved during translation
    """

    def test_basic_markdown_preservation(self):
        """Test preservation of basic markdown elements"""
        original_content = """# Introduction

This is **bold** text and *italic* text.

## Code Block

```python
def hello():
    print("Hello, world!")
```

## List Items

- First item
- Second item
  - Nested item

1. Ordered item
2. Another ordered item

[Link](https://example.com)
"""

        # Preserve structure
        processed_content, preserver = preserve_markdown_structure(original_content)

        # Restore structure
        restored_content = restore_markdown_structure(processed_content, preserver)

        # Validate preservation
        validation_result = validate_markdown_preservation(original_content, restored_content)

        # Verify all elements are preserved
        assert validation_result["code_blocks_preserved"] is True
        assert validation_result["headings_preserved"] is True
        assert validation_result["lists_preserved"] is True
        assert validation_result["links_preserved"] is True
        assert validation_result["formatting_preserved"] is True

        # Overall preservation should be high
        assert validation_result["overall_preservation_rate"] >= 0.95

    def test_complex_markdown_preservation(self):
        """Test preservation of complex markdown structures"""
        complex_content = """# Complex Markdown Test

## Table Example

| Header 1 | Header 2 | Header 3 |
|----------|----------|----------|
| Cell 1   | Cell 2   | Cell 3   |
| Cell 4   | Cell 5   | Cell 6   |

## Blockquotes

> This is a blockquote
> With multiple lines

## Mixed Formatting

This paragraph has **bold**, *italic*, and `inline code` formatting.

### Nested Lists

- Main item
  1. Nested ordered
  2. Another nested
- Another main item
  - Nested unordered
    - Deep nested

## Images (should be preserved)

![Alt text](image.png "Title")

## Headers of Various Levels

### Header 3
#### Header 4
##### Header 5
###### Header 6
"""

        # Preserve structure
        processed_content, preserver = preserve_markdown_structure(complex_content)

        # Restore structure
        restored_content = restore_markdown_structure(processed_content, preserver)

        # Validate preservation
        validation_result = validate_markdown_preservation(complex_content, restored_content)

        # All elements should be preserved
        assert validation_result["tables_preserved"] is True
        assert validation_result["blockquotes_preserved"] is True
        assert validation_result["images_preserved"] is True
        assert validation_result["nested_lists_preserved"] is True
        assert validation_result["all_headers_preserved"] is True

        # Overall preservation should be high
        assert validation_result["overall_preservation_rate"] >= 0.95

    def test_code_block_preservation(self):
        """Test that code blocks remain unchanged during translation process"""
        content_with_code = """# Code Block Test

Regular text content.

```javascript
function greet(name) {
    // This is a comment
    return `Hello, ${name}!`;
}
```

More regular text.

```bash
# This is a bash script
echo "Hello, world!"
```

Final text.
"""

        # Preserve structure
        processed_content, preserver = preserve_markdown_structure(content_with_code)

        # Verify code blocks are extracted
        assert len(preserver.code_blocks) >= 2  # Should have at least 2 code blocks

        # Restore structure
        restored_content = restore_markdown_structure(processed_content, preserver)

        # Validate preservation
        validation_result = validate_markdown_preservation(content_with_code, restored_content)

        # Code blocks should be perfectly preserved
        assert validation_result["code_blocks_preserved"] is True
        assert validation_result["code_blocks_exact_match"] is True
        assert validation_result["overall_preservation_rate"] >= 0.95

    def test_special_characters_preservation(self):
        """Test preservation of special markdown characters and symbols"""
        special_content = r"""# Special Characters Test

Escaped characters: \> \* \*\* \` \[ \]

Mathematical expressions: $x = y + z$, $$\int_0^1 x dx$$

Special symbols: © ® ™ § ¶ † ‡ • … ‰

[Reference Link](#section-reference)

Footnotes[^1]

[^1]: This is a footnote.

Abbreviations:
*[HTML]: Hyper Text Markup Language
*[W3C]: World Wide Web Consortium
"""

        # Preserve structure
        processed_content, preserver = preserve_markdown_structure(special_content)

        # Restore structure
        restored_content = restore_markdown_structure(processed_content, preserver)

        # Validate preservation
        validation_result = validate_markdown_preservation(special_content, restored_content)

        # All special elements should be preserved
        assert validation_result["special_chars_preserved"] is True
        assert validation_result["math_expressions_preserved"] is True
        assert validation_result["footnotes_preserved"] is True
        assert validation_result["abbreviations_preserved"] is True
        assert validation_result["overall_preservation_rate"] >= 0.95

    def test_malformed_markdown_handling(self):
        """Test safe handling of malformed markdown"""
        malformed_content = """# Malformed Markdown Test

Some normal content.

[[Unclosed bracket content]]

```python
def unclosed_code_block():
    print("No closing")

## Incomplete list
- Item without proper closure

# Header without space
Malformed#Header

[Broken link without closing
"""

        # Should handle malformed content without errors
        try:
            processed_content, preserver = preserve_markdown_structure(malformed_content)

            # Restore structure
            restored_content = restore_markdown_structure(processed_content, preserver)

            # Validate that the process completed without crashing
            validation_result = validate_markdown_preservation(malformed_content, restored_content)

            # Even with malformed content, we should get a valid result
            assert validation_result is not None
            assert "overall_preservation_rate" in validation_result

        except Exception as e:
            # If there's an error, it should be handled gracefully
            assert False, f"Malformed markdown handling failed with error: {str(e)}"

    def test_preservation_statistics(self):
        """Test that preservation statistics are calculated correctly"""
        test_content = """# Test Content

Regular paragraph.

## Code Example

```python
def test_function():
    return "test"
```

## List Example

- Item 1
- Item 2

[Link](https://example.com)
"""

        # Preserve and restore structure
        processed_content, preserver = preserve_markdown_structure(test_content)
        restored_content = restore_markdown_structure(processed_content, preserver)

        # Validate preservation with detailed statistics
        validation_result = validate_markdown_preservation(test_content, restored_content)

        # Verify that all statistical measures are present and reasonable
        assert "overall_preservation_rate" in validation_result
        assert "elements_count_original" in validation_result
        assert "elements_count_restored" in validation_result
        assert "preserved_elements_count" in validation_result

        # Preservation rate should be very high (≥95%)
        assert validation_result["overall_preservation_rate"] >= 0.95

        # Element counts should be reasonable
        assert validation_result["elements_count_original"] > 0
        assert validation_result["elements_count_restored"] > 0
        assert validation_result["preserved_elements_count"] > 0

    def test_performance_with_large_content(self):
        """Test preservation with large content to ensure performance"""
        # Create large content
        large_content = "# Large Content Test\n\n"
        for i in range(100):
            large_content += f"## Section {i}\n\n"
            large_content += f"This is content for section {i}.\n\n"
            large_content += "```\nCode block content\n```\n\n"
            large_content += f"- List item {i}.1\n"
            large_content += f"- List item {i}.2\n\n"

        # Preserve structure
        import time
        start_time = time.time()
        processed_content, preserver = preserve_markdown_structure(large_content)
        preserve_time = time.time() - start_time

        # Restore structure
        restore_start = time.time()
        restored_content = restore_markdown_structure(processed_content, preserver)
        restore_time = time.time() - restore_start

        # Validate preservation
        validation_result = validate_markdown_preservation(large_content, restored_content)

        # Verify performance - should complete in reasonable time
        # (With simple string operations, this should be very fast)
        assert preserve_time < 5.0  # Less than 5 seconds
        assert restore_time < 5.0  # Less than 5 seconds

        # Verify preservation quality
        assert validation_result["overall_preservation_rate"] >= 0.95

    def test_edge_cases(self):
        """Test edge cases for markdown preservation"""
        # Empty content
        empty_result = validate_markdown_preservation("", "")
        assert empty_result["overall_preservation_rate"] == 1.0  # Perfect preservation of empty content

        # Only code blocks
        code_only = "```\ncode only\n```"
        processed, preserver = preserve_markdown_structure(code_only)
        restored = restore_markdown_structure(processed, preserver)
        code_validation = validate_markdown_preservation(code_only, restored)
        assert code_validation["overall_preservation_rate"] >= 0.95

        # Only headings
        headings_only = "# H1\n## H2\n### H3"
        processed, preserver = preserve_markdown_structure(headings_only)
        restored = restore_markdown_structure(processed, preserver)
        headings_validation = validate_markdown_preservation(headings_only, restored)
        assert headings_validation["overall_preservation_rate"] >= 0.95

        # Unicode content
        unicode_content = "# مثال بالعربية\n\nThis is a test with юникод and 汉字."
        processed, preserver = preserve_markdown_structure(unicode_content)
        restored = restore_markdown_structure(processed, preserver)
        unicode_validation = validate_markdown_preservation(unicode_content, restored)
        # Unicode preservation might be slightly lower due to character differences, but should still be high
        assert unicode_validation["overall_preservation_rate"] >= 0.90


# Run tests if this file is executed directly
if __name__ == "__main__":
    import asyncio

    test_suite = TestMarkdownPreservation()

    # Run all tests
    print("Testing basic markdown preservation...")
    test_suite.test_basic_markdown_preservation()
    print("✓ Basic markdown preservation test passed")

    print("Testing complex markdown preservation...")
    test_suite.test_complex_markdown_preservation()
    print("✓ Complex markdown preservation test passed")

    print("Testing code block preservation...")
    test_suite.test_code_block_preservation()
    print("✓ Code block preservation test passed")

    print("Testing special characters preservation...")
    test_suite.test_special_characters_preservation()
    print("✓ Special characters preservation test passed")

    print("Testing malformed markdown handling...")
    test_suite.test_malformed_markdown_handling()
    print("✓ Malformed markdown handling test passed")

    print("Testing preservation statistics...")
    test_suite.test_preservation_statistics()
    print("✓ Preservation statistics test passed")

    print("Testing performance with large content...")
    test_suite.test_performance_with_large_content()
    print("✓ Performance with large content test passed")

    print("Testing edge cases...")
    test_suite.test_edge_cases()
    print("✓ Edge cases test passed")

    print("\nAll markdown preservation tests passed! ✓")
    print("Verified ≥95% markdown structure preservation requirement.")