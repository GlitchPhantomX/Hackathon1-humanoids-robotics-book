#!/usr/bin/env python3
"""
Script to apply custom styling to Docusaurus markdown files.
This script transforms standard markdown headings to JSX with className attributes
and adds custom styling elements as specified in the requirements.
"""

import os
import re
import shutil
from pathlib import Path


def backup_file(file_path):
    """Create a backup of the file."""
    backup_path = file_path + '.backup'
    shutil.copy2(file_path, backup_path)
    print(f"Created backup: {backup_path}")


def has_styling_already(content):
    """Check if the file already has the custom styling applied."""
    # Check for presence of className attributes that indicate styling is applied
    has_h1_styling = '<h1 className="main-heading"' in content
    has_h2_styling = '<h2 className="second-heading"' in content
    has_underline = '<div className="underline-class"></div>' in content

    return has_h1_styling or has_h2_styling or has_underline


def apply_styling_to_file(file_path):
    """Apply styling transformations to a single file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Skip if styling is already applied
    if has_styling_already(content):
        print(f"Skipping {file_path} - styling already applied")
        return False

    # Create backup
    backup_file(file_path)

    # Parse the content to separate frontmatter, imports, and main content
    lines = content.split('\n')
    frontmatter = []
    imports = []
    main_content = []

    # State tracking
    in_frontmatter = False
    after_frontmatter = False  # Tracks if we're after the frontmatter closing ---
    in_imports_section = False  # Tracks if we're in the import section

    i = 0
    while i < len(lines):
        line = lines[i]

        # Check for frontmatter start
        if line.strip() == '---' and not in_frontmatter and not after_frontmatter:
            in_frontmatter = True
            frontmatter.append(line)
            i += 1
            continue

        # Process frontmatter content
        if in_frontmatter:
            frontmatter.append(line)
            # Check if this is the closing --- of frontmatter (not the opening one)
            if line.strip() == '---' and len(frontmatter) > 1:
                in_frontmatter = False
                after_frontmatter = True  # Now we're after frontmatter
            i += 1
            continue

        # After frontmatter, look for import statements
        if after_frontmatter and not in_imports_section:
            if line.strip().startswith('import '):
                in_imports_section = True
                imports.append(line)
                i += 1
                continue
            elif line.strip() == '':  # Empty line after frontmatter is OK
                imports.append(line)  # Add to imports section for spacing
                i += 1
                continue
            else:
                # First non-import, non-empty line after frontmatter starts main content
                after_frontmatter = False
                in_imports_section = False
                main_content.append(line)
                i += 1
                continue

        # If we're in imports section
        if in_imports_section:
            if line.strip().startswith('import '):
                imports.append(line)
                i += 1
                continue
            elif line.strip() == '':  # Empty lines are OK in imports section
                imports.append(line)
                i += 1
                continue
            else:
                # Non-import content starts main content
                in_imports_section = False
                main_content.append(line)
                i += 1
                continue

        # All remaining content goes to main_content
        main_content.append(line)
        i += 1

    # Process main content to apply styling
    processed_content = process_content(main_content)

    # Reassemble the file
    new_content = '\n'.join(frontmatter)
    if imports:
        new_content += '\n' + '\n'.join(imports)
    if processed_content:
        new_content += '\n' + '\n'.join(processed_content)

    # Write the modified content back to the file
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"Applied styling to: {file_path}")
    return True


def process_content(content_lines):
    """Process the main content to apply styling transformations."""
    result = []
    i = 0
    in_code_block = False
    code_block_language = ""

    while i < len(content_lines):
        line = content_lines[i]

        # Check if entering or exiting a code block
        if line.strip().startswith('```'):
            in_code_block = not in_code_block
            # Extract language if present
            parts = line.strip().split('```')
            if len(parts) > 1 and parts[1]:
                code_block_language = parts[1]
            else:
                code_block_language = ""
            result.append(line)
            i += 1
            continue

        # If we're in a code block, don't transform anything
        if in_code_block:
            result.append(line)
            i += 1
            continue

        # Transform H1 headings (but not if they're in a code context like Python comments)
        # Only match lines that start exactly with '# ' (space after #) and have no leading whitespace
        if line.strip().startswith('# ') and line.startswith('# '):
            # Extract the heading text after '# '
            heading_text = line[2:].strip()  # Remove '# ' prefix
            result.append(f'# <h1 className="main-heading">{heading_text}</h1>')
            result.append('<div className="underline-class"></div>')
            i += 1
            continue

        # Transform H2 headings (but not if they're in a code context)
        if line.strip().startswith('## ') and line.startswith('## '):
            heading_text = line[3:].strip()  # Remove '## ' prefix
            result.append(f'<h2 className="second-heading">')
            result.append(heading_text)
            result.append('</h2>')
            result.append('<div className="underline-class"></div>')
            i += 1
            continue

        # Transform H3 headings (but not if they're in a code context)
        if line.strip().startswith('### ') and line.startswith('### '):
            heading_text = line[4:].strip()  # Remove '### ' prefix
            result.append(f'<h3 className="third-heading">')
            result.append(f'- {heading_text}')
            result.append('</h3>')
            result.append('<div className="underline-class"></div>')
            i += 1
            continue

        # Transform H4 headings (but not if they're in a code context)
        if line.strip().startswith('#### ') and line.startswith('#### '):
            heading_text = line[5:].strip()  # Remove '#### ' prefix
            result.append(f'<h4 className="fourth-heading">')
            result.append(heading_text)
            result.append('</h4>')
            result.append('<div className="underline-class"></div>')
            i += 1
            continue

        # Transform list items (level 1)
        list1_match = re.match(r'^(\s*)- (.+)$', line)
        if list1_match:
            indent = list1_match.group(1)
            rest_of_line = list1_match.group(2)
            # Only transform if it doesn't already start with a custom bullet
            if not rest_of_line.lstrip().startswith(('•', '➤', '▸', '◦', '▹', '▸', '➛', '➜', '→', '➟')):
                result.append(f'{indent}- • {rest_of_line}')
            else:
                result.append(line)  # Already has custom bullet
            i += 1
            continue

        # Transform list items (level 2)
        list2_match = re.match(r'^(\s{2,})- (.+)$', line)
        if list2_match:
            indent = list2_match.group(1)
            rest_of_line = list2_match.group(2)
            # Only transform if it doesn't already start with a custom bullet
            if not rest_of_line.lstrip().startswith(('•', '➤', '▸', '◦', '▹', '▸', '➛', '➜', '→', '➟')):
                result.append(f'{indent}- ▸ {rest_of_line}')
            else:
                result.append(line)  # Already has custom bullet
            i += 1
            continue

        # Handle horizontal rules (but not inside code blocks)
        if line.strip() == '---':
            result.append('<div className="border-line"></div>')
            result.append('---')
            i += 1
            continue

        # Add regular lines as-is
        result.append(line)
        i += 1

    return result


def main():
    """Main function to process all markdown files in the docs directory."""
    docs_dir = Path("docs")

    # Find all .md and .mdx files in the docs directory
    md_files = list(docs_dir.rglob("*.md"))
    mdx_files = list(docs_dir.rglob("*.mdx"))
    all_files = md_files + mdx_files

    processed_count = 0

    for file_path in all_files:
        try:
            if apply_styling_to_file(str(file_path)):
                processed_count += 1
        except Exception as e:
            print(f"Error processing {file_path}: {str(e)}")

    print(f"\nProcessing complete! {processed_count} files were styled.")


if __name__ == "__main__":
    main()