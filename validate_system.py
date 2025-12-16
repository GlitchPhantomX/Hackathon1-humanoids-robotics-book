#!/usr/bin/env python3
"""
Validation script for the Reusable Intelligence System
This script validates that all components of the system are properly implemented
and meet the requirements specified in the tasks document.
"""

import os
import json
import yaml
from pathlib import Path

def validate_subagent_structure(subagent_name, expected_files):
    """Validate that a subagent has all required files."""
    subagent_path = Path(f"subagents/{subagent_name}")
    print(f"\nValidating {subagent_name} subagent...")

    missing_files = []
    for expected_file in expected_files:
        file_path = subagent_path / expected_file
        if not file_path.exists():
            missing_files.append(str(file_path))
        else:
            print(f"  [PASS] {expected_file}")

    if missing_files:
        print(f"  [FAIL] Missing files: {missing_files}")
        return False
    else:
        print(f"  [PASS] All files present for {subagent_name}")
        return True

def validate_skill_structure(skill_name, expected_files):
    """Validate that a skill has all required files."""
    skill_path = Path(f"skills/{skill_name}")
    print(f"\nValidating {skill_name} skill...")

    missing_files = []
    for expected_file in expected_files:
        file_path = skill_path / expected_file
        if not file_path.exists():
            missing_files.append(str(file_path))
        else:
            print(f"  [PASS] {expected_file}")

    if missing_files:
        print(f"  [FAIL] Missing files: {missing_files}")
        return False
    else:
        print(f"  [PASS] All files present for {skill_name}")
        return True

def validate_file_content(file_path, required_content_substrings):
    """Validate that a file contains required content."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        missing_content = []
        for required in required_content_substrings:
            if required not in content:
                missing_content.append(required)

        if missing_content:
            print(f"  [FAIL] Missing content in {file_path}: {missing_content}")
            return False
        else:
            return True
    except Exception as e:
        print(f"  [FAIL] Error reading {file_path}: {e}")
        return False

def main():
    print("Reusable Intelligence System - Validation Script")
    print("=" * 50)

    # Validate Technical Writer Subagent
    tech_writer_files = [
        "SUBAGENT.md",
        "config.yaml",
        "system-prompt.txt",
        "README.md",
        "examples/sample-chapter.md",
        "examples/chapter-generation-log.json",
        "examples/usage-statistics.md"
    ]

    tech_writer_valid = validate_subagent_structure("technical-writer", tech_writer_files)

    # Validate Code Generator Subagent
    code_gen_files = [
        "SUBAGENT.md",
        "config.yaml",
        "system-prompt.txt",
        "README.md",
        "examples/ros2-examples/node_example.py",
        "examples/ros2-examples/publisher_example.py",
        "examples/ros2-examples/subscriber_example.py",
        "examples/generation-log.json",
        "examples/usage-statistics.md"
    ]

    code_gen_valid = validate_subagent_structure("code-generator", code_gen_files)

    # Validate RAG Specialist Subagent
    rag_spec_files = [
        "SUBAGENT.md",
        "config.yaml",
        "system-prompt.txt",
        "README.md",
        "examples/rag_implementation.py",
        "examples/config.yaml",
        "examples/generation-log.json",
        "examples/usage-statistics.md"
    ]

    rag_spec_valid = validate_subagent_structure("rag-specialist", rag_spec_files)

    # Validate Docusaurus Chapter Creator Skill
    docusaurus_skill_files = [
        "SKILL.md",
        "config.yaml",
        "system-prompt.txt",
        "README.md",
        "examples/ros2_services_example.mdx",
        "examples/generation-log.json",
        "examples/usage-statistics.md"
    ]

    docusaurus_valid = validate_skill_structure("docusaurus-chapter-creator", docusaurus_skill_files)

    # Validate ROS2 Code Validator Skill
    ros2_validator_files = [
        "SKILL.md",
        "config.yaml",
        "system-prompt.txt",
        "README.md",
        "examples/sample_ros2_code.py",
        "examples/validation_report_example.json",
        "examples/generation-log.json",
        "examples/usage-statistics.md"
    ]

    ros2_validator_valid = validate_skill_structure("ros2-code-validator", ros2_validator_files)

    # Validate RAG Deployer Skill
    rag_deployer_files = [
        "SKILL.md",
        "config.yaml",
        "system-prompt.txt",
        "README.md",
        "examples/Dockerfile",
        "examples/k8s-deployment.yaml",
        "examples/terraform-config.tf",
        "examples/generation-log.json",
        "examples/usage-statistics.md"
    ]

    rag_deployer_valid = validate_skill_structure("rag-deployer", rag_deployer_files)

    # Validate main documentation
    print(f"\nValidating main documentation...")
    reusability_path = Path("REUSABILITY.md")
    if reusability_path.exists():
        print(f"  [PASS] REUSABILITY.md exists")
        reusability_valid = validate_file_content(
            reusability_path,
            ["Reusable Intelligence System", "Architecture", "Implementation Evidence"]
        )
    else:
        print(f"  [FAIL] REUSABILITY.md missing")
        reusability_valid = False

    # Summary
    print(f"\n" + "=" * 50)
    print(f"VALIDATION SUMMARY")
    print(f"=" * 50)

    all_valid = all([
        tech_writer_valid,
        code_gen_valid,
        rag_spec_valid,
        docusaurus_valid,
        ros2_validator_valid,
        rag_deployer_valid,
        reusability_valid
    ])

    print(f"Technical Writer Subagent: {'[PASS]' if tech_writer_valid else '[FAIL]'}")
    print(f"Code Generator Subagent: {'[PASS]' if code_gen_valid else '[FAIL]'}")
    print(f"RAG Specialist Subagent: {'[PASS]' if rag_spec_valid else '[FAIL]'}")
    print(f"Docusaurus Chapter Creator Skill: {'[PASS]' if docusaurus_valid else '[FAIL]'}")
    print(f"ROS2 Code Validator Skill: {'[PASS]' if ros2_validator_valid else '[FAIL]'}")
    print(f"RAG Deployer Skill: {'[PASS]' if rag_deployer_valid else '[FAIL]'}")
    print(f"REUSABILITY.md Documentation: {'[PASS]' if reusability_valid else '[FAIL]'}")

    print(f"\nOverall System Validation: {'[PASS]' if all_valid else '[FAIL]'}")

    if all_valid:
        print(f"\n[SUCCESS] All components validated successfully!")
        print(f"The Reusable Intelligence System is complete and ready for use.")
    else:
        print(f"\n[WARNING] Some components failed validation.")
        print(f"Please review the validation output above and address any issues.")

    return all_valid

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)