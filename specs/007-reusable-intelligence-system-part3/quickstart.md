# Quickstart Guide: Reusable Intelligence System - Part 3

## Overview

This quickstart guide will help you get up and running with the Reusable Intelligence System featuring 3 Claude Code Subagents and 3 Agent Skills for the Physical AI & Humanoid Robotics Textbook project.

## Prerequisites

### System Requirements
- Windows, macOS, or Linux
- Python 3.11+ installed
- Node.js 18+ installed (for Docusaurus integration)
- Claude Code CLI installed and configured
- Access to Anthropic API (for Claude models)
- Git installed for version control

### Environment Setup
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Install Node.js dependencies (if using Docusaurus):
   ```bash
   cd physical-ai-robotics-textbook/docusaurus
   npm install
   ```

4. Set up environment variables:
   ```bash
   # Create .env file with required API keys
   echo "ANTHROPIC_API_KEY=your_api_key_here" > .env
   echo "OPENAI_API_KEY=your_openai_key_here" >> .env
   echo "QDRANT_URL=your_qdrant_url_here" >> .env
   ```

## Installation

### 1. Subagent Installation

The subagents are pre-configured and ready to use. You can find them in the following directories:

```
subagents/
├── technical-writer/
├── code-generator/
└── rag-specialist/
```

Each subagent directory contains:
- `SUBAGENT.md` - Comprehensive documentation
- `config.yaml` - Configuration settings
- `system-prompt.txt` - AI system prompt
- `examples/` - Usage examples and statistics
- `README.md` - Quick start instructions

### 2. Skill Installation

The skills are also pre-configured and ready to use:

```
skills/
├── docusaurus-chapter-creator/
├── ros2-code-validator/
└── rag-deployer/
```

Each skill directory contains:
- `SKILL.md` - Complete skill documentation
- `examples/` - Input/output examples
- `tests/` - Test cases and validation
- `templates/` - Template files (if applicable)
- `README.md` - Quick start instructions

## Quick Start Examples

### 1. Using the Technical Writer Subagent

Generate a technical chapter about ROS2 Publishers and Subscribers:

```bash
# Navigate to the repository root
cd C:\new - Copy

# Use the technical writer subagent
claude-code subagent:technical-writer \
  --topic "ROS 2 Publishers and Subscribers" \
  --module "Module 1: ROS 2 Fundamentals" \
  --length 2000 \
  --code-examples \
  --diagrams
```

**Expected Output:**
- Creates a new MDX file in the appropriate location
- Contains 2000+ words of technical content
- Includes 2-4 Python code examples
- Contains Mermaid diagrams for visualization
- Properly formatted with Docusaurus frontmatter

### 2. Using the Code Generator Subagent

Generate a ROS2 publisher node:

```bash
claude-code subagent:code-generator \
  --purpose "Create a ROS2 publisher for laser scan data" \
  --language python \
  --include-comments \
  --include-error-handling
```

**Expected Output:**
- Creates a complete ROS2 publisher node
- Includes proper error handling
- Contains comprehensive comments
- Follows ROS2 conventions
- Ready to run with minimal modifications

### 3. Using the RAG Specialist Subagent

Create a RAG backend for your documentation:

```bash
claude-code subagent:rag-specialist \
  --project-name "textbook-chatbot" \
  --documents-source "./docs" \
  --openai-model "text-embedding-3-small" \
  --qdrant-collection "textbook_chapters" \
  --deployment-target docker
```

**Expected Output:**
- Creates a complete FastAPI project
- Sets up Qdrant vector database integration
- Implements document ingestion pipeline
- Creates query endpoints with retrieval
- Generates Docker configuration
- Includes comprehensive tests

### 4. Using the Docusaurus Chapter Creator Skill

Generate a chapter from an outline:

```bash
# Create an input JSON file (chapter-outline.json)
cat > chapter-outline.json << EOF
{
  "title": "Introduction to NVIDIA Isaac Sim",
  "module": "Module 3: NVIDIA Isaac",
  "key_concepts": [
    "Photorealistic simulation",
    "USD (Universal Scene Description)",
    "Physics engine integration"
  ],
  "target_audience": "graduate_students",
  "desired_length": 2500,
  "include_code_examples": true,
  "include_diagrams": true,
  "related_chapters": [
    "/docs/module3/isaac-intro",
    "/docs/module2/gazebo-basics"
  ]
}
EOF

# Run the skill
claude-code skill:docusaurus-chapter-creator chapter-outline.json
```

### 5. Using the ROS2 Code Validator Skill

Validate a ROS2 Python file:

```bash
# Validate a ROS2 node file
claude-code skill:ros2-code-validator my_ros2_node.py
```

**Expected Output:**
- Validation report in JSON format
- List of errors and warnings
- Improvement suggestions
- Quality metrics

### 6. Using the RAG Deployer Skill

Deploy a complete RAG system:

```bash
# Create a deployment configuration (deployment-config.json)
cat > deployment-config.json << EOF
{
  "project_name": "textbook-rag",
  "documents_source": "./docs",
  "openai_model": "text-embedding-3-small",
  "qdrant_collection": "textbook_content",
  "deployment_target": "docker",
  "cors_origins": ["http://localhost:3000"]
}
EOF

# Deploy the RAG system
claude-code skill:rag-deployer deployment-config.json
```

## Configuration

### Customizing Subagent Settings

You can modify the behavior of subagents by editing their configuration files:

1. **Technical Writer Configuration** (`subagents/technical-writer/config.yaml`):
   - Adjust temperature for creativity vs. consistency
   - Modify word count targets
   - Change quality thresholds
   - Update domain expertise

2. **Code Generator Configuration** (`subagents/code-generator/config.yaml`):
   - Change model parameters
   - Update code standards
   - Modify validation rules
   - Adjust output formats

3. **RAG Specialist Configuration** (`subagents/rag-specialist/config.yaml`):
   - Adjust performance targets
   - Modify architecture patterns
   - Update quality requirements
   - Change integration settings

### Customizing Skill Settings

Skills can be configured through their respective configuration files:

1. **Docusaurus Chapter Creator** (`skills/docusaurus-chapter-creator/config.yaml`):
   - Modify word count parameters
   - Change quality checks
   - Update performance settings

2. **ROS2 Code Validator** (`skills/ros2-code-validator/config.yaml`):
   - Adjust validation levels
   - Update ROS2 conventions
   - Modify quality metrics

3. **RAG Deployer** (`skills/rag-deployer/config.yaml`):
   - Change deployment targets
   - Update performance targets
   - Modify component selection

## Best Practices

### For Technical Writers
- Use specific, focused topics for better results
- Provide clear learning objectives
- Include prerequisite knowledge
- Specify target audience level
- Request appropriate code examples for the topic

### For Code Generation
- Be specific about the purpose and requirements
- Specify the ROS2 distribution if relevant
- Request appropriate complexity level
- Include error handling for production code
- Verify generated code before deployment

### For RAG Systems
- Organize documents with clear structure
- Use appropriate embedding models for your content
- Test query performance with real questions
- Monitor resource usage in production
- Plan for scaling as document count grows

## Troubleshooting

### Common Issues

1. **API Rate Limits**
   - Wait before making additional requests
   - Check your API quota
   - Consider upgrading your plan if needed

2. **Quality Issues**
   - Review the system prompt for the subagent
   - Adjust configuration parameters
   - Provide more specific input requirements
   - Validate output before using in production

3. **Configuration Errors**
   - Check YAML syntax in configuration files
   - Verify all required parameters are present
   - Ensure environment variables are set correctly

### Performance Tips

1. **Caching**: Enable caching in configuration for repeated operations
2. **Batch Processing**: Process multiple items together when possible
3. **Optimal Prompts**: Use specific, detailed prompts for better results
4. **Resource Management**: Monitor system resources during generation

## Next Steps

### Immediate Actions
1. Try the quickstart examples above
2. Review the documentation in each subagent/skill directory
3. Customize configurations for your specific needs
4. Run the test suites to verify functionality

### Advanced Usage
1. Integrate components into your CI/CD pipeline
2. Create custom templates for your specific use cases
3. Extend the system with additional subagents or skills
4. Monitor and optimize performance based on usage metrics

### Production Deployment
1. Set up proper monitoring and logging
2. Implement proper error handling and recovery
3. Plan for scaling based on your usage patterns
4. Establish backup and recovery procedures

## Support

### Documentation
- Each component has comprehensive documentation in its respective directory
- Check the `REUSABILITY.md` file for overall system documentation
- Review the `spec.md` and `plan.md` files for detailed specifications

### Examples
- Look in the `examples/` directories for usage examples
- Check the `tests/` directories for validation examples
- Review the `usage-statistics.md` files for real-world usage patterns

### Getting Help
- Review the README.md files in each component directory
- Check the system logs for detailed error information
- Contact the development team for advanced support needs

## Conclusion

The Reusable Intelligence System is now ready for use. With 3 specialized subagents and 3 automation skills, you have a comprehensive toolkit for generating high-quality technical content, production-ready code, and complete backend systems. The system is designed for reusability across multiple projects and has been validated with measurable impact on development efficiency.

Start with the quickstart examples and gradually explore more advanced features as you become familiar with the system. The modular design allows you to use individual components or combine them for more complex workflows.