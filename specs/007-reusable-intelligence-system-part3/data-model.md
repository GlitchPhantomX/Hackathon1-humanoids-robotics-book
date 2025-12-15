# Data Model: Reusable Intelligence System - Part 3

## Overview

This document defines the data models used by the Reusable Intelligence System, including the 3 Claude Code Subagents and 3 Agent Skills for the Physical AI & Humanoid Robotics Textbook project.

## System Architecture

### Core Components
```
Reusable Intelligence System
├── Subagents Layer
│   ├── Technical Writer
│   ├── Code Generator
│   └── RAG Specialist
├── Skills Layer
│   ├── Docusaurus Chapter Creator
│   ├── ROS2 Code Validator
│   └── RAG Deployer
└── Evidence Layer
    ├── Generation Logs
    ├── Usage Statistics
    └── Quality Metrics
```

## Data Models

### 1. Subagent Configuration Model

#### Common Subagent Configuration
```yaml
subagent:
  name: string                    # Unique identifier for the subagent
  version: string                 # Semantic version (e.g., "1.0.0")
  type: string                    # Type of subagent (content_generation, code_generation, etc.)
  description: string             # Brief description of purpose
  model:
    provider: string              # AI provider (e.g., "anthropic")
    name: string                  # Model name (e.g., "claude-sonnet-4-20250514")
    temperature: number           # Creativity parameter (0.0-1.0)
    max_tokens: number            # Maximum tokens in response
    top_p: number                 # Nucleus sampling parameter
  capabilities: [string]          # List of capabilities
  domains: [string]               # Technical domains of expertise
  output:
    formats: [string]             # Supported output formats (mdx, markdown, python, etc.)
    default_format: string        # Default output format
    structure:                    # Content structure parameters
      min_word_count: number
      max_word_count: number
      target_word_count: number
    components:                   # Required content components
      code_examples_min: number
      code_examples_max: number
      diagrams_preferred: boolean
      callout_boxes: boolean
      cross_references: boolean
  quality_checks:                 # Quality assurance configuration
    enabled: boolean
    checks: [string]              # List of quality checks to perform
    thresholds:                   # Quality thresholds
      min_quality_score: number
      min_readability: number
      max_cyclomatic_complexity: number
  validation:                     # Validation rules
    heading:
      h1_count: number            # Expected number of H1 headings
      h1_matches_title: boolean   # Whether H1 matches document title
      max_nesting_level: number   # Maximum heading level (1-6)
    code:
      syntax_check: boolean       # Whether to check code syntax
      language_tag_required: boolean # Whether language tags required
      include_comments: boolean   # Whether comments required
    links:
      validate_internal: boolean  # Whether to validate internal links
      validate_external: boolean  # Whether to validate external links
  performance:                    # Performance configuration
    timeout_seconds: number       # Maximum execution time
    retry_attempts: number        # Number of retry attempts
    cache_results: boolean        # Whether to cache results
  logging:                        # Logging configuration
    level: string                 # Log level (DEBUG, INFO, WARN, ERROR)
    log_file: string              # Log file path
    include_timestamps: boolean   # Whether to include timestamps
    include_metrics: boolean      # Whether to include performance metrics
  integrations: [string]          # List of integrations
  usage_tracking:                 # Usage tracking configuration
    enabled: boolean
    metrics_file: string          # File to store metrics
    log_file: string              # File to store generation logs
```

#### Technical Writer Subagent Configuration
```yaml
subagent:
  name: technical-writer
  version: 1.0.0
  type: content_generation
  description: "Specialized technical documentation generator for Physical AI and Robotics"
  model:
    provider: anthropic
    name: claude-sonnet-4-20250514
    temperature: 0.3
    max_tokens: 4000
    top_p: 0.9
  capabilities:
    - technical_writing
    - code_examples
    - diagram_generation
    - cross_referencing
    - markdown_formatting
    - mdx_formatting
  domains:
    - ros2
    - gazebo
    - isaac_sim
    - physical_ai
    - robotics
    - python
    - cpp
    - urdf
    - sdf
  output:
    formats:
      - mdx
      - markdown
    default_format: mdx
    structure:
      min_word_count: 1500
      max_word_count: 3000
      target_word_count: 2000
    components:
      code_examples_min: 2
      code_examples_max: 6
      diagrams_preferred: true
      callout_boxes: true
      cross_references: true
  quality_checks:
    enabled: true
    checks:
      - technical_accuracy
      - code_syntax
      - link_validation
      - heading_hierarchy
      - markdown_linting
      - readability_score
    thresholds:
      min_quality_score: 8.0
      min_readability: 7.0
      max_cyclomatic_complexity: 10
  validation:
    heading:
      h1_count: 1
      h1_matches_title: true
      max_nesting_level: 6
    code:
      syntax_check: true
      language_tag_required: true
      include_comments: true
    links:
      validate_internal: true
      validate_external: false
  performance:
    timeout_seconds: 3600
    retry_attempts: 3
    cache_results: true
  logging:
    level: INFO
    log_file: technical-writer.log
    include_timestamps: true
    include_metrics: true
  integrations:
    - docusaurus
    - github
    - markdown_linters
    - vale
  usage_tracking:
    enabled: true
    metrics_file: examples/usage-statistics.md
    log_file: examples/chapter-generation-log.json
```

#### Code Generator Subagent Configuration
```yaml
subagent:
  name: code-generator
  version: 1.0.0
  type: code_generation
  description: "Production-ready code generator for ROS2 and robotics applications"
  model:
    provider: anthropic
    name: claude-sonnet-4-20250514
    temperature: 0.2
    max_tokens: 2000
    top_p: 0.8
  capabilities:
    - ros2_python
    - urdf_xml
    - isaac_sim_python
    - error_handling
  languages:
    - python
    - xml
    - yaml
  code_standards:
    python:
      - pep8
      - type_hints
      - docstrings
      - black_formatting
  validation:
    - syntax_check
    - import_validation
    - ros2_conventions
    - security_scan
  quality_checks:
    enabled: true
    checks:
      - syntax_check
      - pep8_compliance
      - ros2_conventions
      - docstring_coverage
      - type_hint_coverage
    thresholds:
      min_quality_score: 9.0
      max_cyclomatic_complexity: 10
      min_docstring_coverage: 80
  performance:
    timeout_seconds: 1800
    retry_attempts: 3
    cache_results: true
  logging:
    level: INFO
    log_file: code-generator.log
    include_timestamps: true
    include_metrics: true
  integrations:
    - python_linters
    - ros2_tools
    - pytest
  usage_tracking:
    enabled: true
    metrics_file: examples/usage-statistics.md
    log_file: examples/generation-log.json
```

#### RAG Specialist Subagent Configuration
```yaml
subagent:
  name: rag-specialist
  version: 1.0.0
  type: backend_generation
  description: "RAG chatbot backend specialist for documentation and knowledge bases"
  model:
    provider: anthropic
    name: claude-sonnet-4-20250514
    temperature: 0.3
    max_tokens: 3000
    top_p: 0.9
  capabilities:
    - fastapi_development
    - qdrant_integration
    - openai_embeddings
    - api_design
    - async_programming
  technologies:
    - fastapi
    - qdrant
    - openai
    - pydantic
    - asyncio
  architecture_patterns:
    - rest_api
    - vector_search
    - rag_pipeline
    - async_io
  quality_requirements:
    - type_safety
    - error_handling
    - logging
    - documentation
    - testing
  performance_targets:
    query_response_time_ms: 500
    retrieval_accuracy_percent: 85
    concurrent_users: 100
  quality_checks:
    enabled: true
    checks:
      - type_hints
      - error_handling
      - security_validation
      - performance_check
      - documentation_coverage
    thresholds:
      min_quality_score: 8.5
      max_response_time_ms: 500
      min_test_coverage: 80
  performance:
    timeout_seconds: 3600
    retry_attempts: 3
    cache_results: true
  logging:
    level: INFO
    log_file: rag-specialist.log
    include_timestamps: true
    include_metrics: true
  integrations:
    - fastapi
    - qdrant_client
    - openai_sdk
    - pytest
  usage_tracking:
    enabled: true
    metrics_file: examples/usage-statistics.md
    log_file: examples/deployment-log.json
```

### 2. Skill Configuration Model

#### Common Skill Configuration
```yaml
skill:
  name: string                    # Unique identifier for the skill
  version: string                 # Semantic version (e.g., "1.0.0")
  input_format: string            # Format of input data (json, yaml, etc.)
  output_format: string           # Format of output data (mdx, python, etc.)
  description: string             # Brief description of purpose
  parameters:                     # Configuration parameters
    [parameter_name]: value       # Specific parameters for this skill
  quality_checks: [string]        # List of quality checks to perform
  validation:                     # Validation rules
    input_validation: boolean     # Whether to validate input
    output_validation: boolean    # Whether to validate output
    format_validation: boolean    # Whether to validate format
  performance:                    # Performance configuration
    timeout_seconds: number       # Maximum execution time
    retry_attempts: number        # Number of retry attempts
  logging:                        # Logging configuration
    level: string                 # Log level (DEBUG, INFO, WARN, ERROR)
    log_file: string              # Log file path
  integrations: [string]          # List of integrations
  usage_tracking:                 # Usage tracking configuration
    enabled: boolean
    metrics_file: string          # File to store metrics
    log_file: string              # File to store usage logs
```

#### Docusaurus Chapter Creator Skill Configuration
```yaml
skill:
  name: docusaurus-chapter-creator
  version: 1.0.0
  input_format: json
  output_format: mdx
  description: "Automated generation of complete Docusaurus chapters from topic outlines"
  parameters:
    min_word_count: 1500
    max_word_count: 3000
    heading_levels: 6
    code_examples_min: 2
    diagrams_preferred: true
  quality_checks:
    - frontmatter_validation
    - heading_hierarchy
    - code_syntax
    - link_validation
    - markdown_linting
  validation:
    input_validation: true
    output_validation: true
    format_validation: true
  performance:
    timeout_seconds: 1800
    retry_attempts: 2
  logging:
    level: INFO
    log_file: docusaurus-chapter-creator.log
  integrations:
    - docusaurus
    - markdown_linters
  usage_tracking:
    enabled: true
    metrics_file: examples/usage-statistics.md
    log_file: examples/generation-log.json
```

#### ROS2 Code Validator Skill Configuration
```yaml
skill:
  name: ros2-code-validator
  version: 1.0.0
  input_format: python
  output_format: json
  description: "Validate, test, and improve ROS2 Python code with comprehensive analysis"
  parameters:
    validation_level: "standard"  # basic, standard, or strict
    ros2_distribution: "humble"   # Target ROS2 distribution
  validation_levels:
    basic:
      - syntax_check
      - import_validation
    standard:
      - basic
      - naming_conventions
      - error_handling
      - docstrings
    strict:
      - standard
      - type_hints
      - complexity_analysis
      - performance_checks
  ros2_conventions:
    - snake_case_names
    - proper_node_initialization
    - proper_shutdown
    - qos_profile_usage
  quality_metrics:
    max_cyclomatic_complexity: 10
    min_docstring_coverage: 80
    max_line_length: 100
  quality_checks:
    - syntax_check
    - naming_conventions
    - docstring_validation
    - import_validation
  validation:
    input_validation: true
    output_validation: true
    format_validation: true
  performance:
    timeout_seconds: 300
    retry_attempts: 2
  logging:
    level: INFO
    log_file: ros2-code-validator.log
  integrations:
    - python_linters
    - ros2_tools
  usage_tracking:
    enabled: true
    metrics_file: examples/usage-statistics.md
    log_file: examples/validation-log.json
```

#### RAG Deployer Skill Configuration
```yaml
skill:
  name: rag-deployer
  version: 1.0.0
  input_format: json
  output_format: project_directory
  description: "End-to-end deployment of RAG chatbot backends"
  components:
    - fastapi_app
    - qdrant_integration
    - openai_embeddings
    - api_endpoints
    - docker_config
    - testing_suite
  api_endpoints:
    - "POST /query (RAG query)"
    - "POST /ingest (Document ingestion)"
    - "GET /health (Health check)"
    - "GET /docs (API documentation)"
  deployment_targets:
    - local
    - docker
    - railway
    - render
    - fly_io
  performance_targets:
    query_response_time_ms: 500
    concurrent_requests: 100
    uptime_percent: 99.9
  quality_checks:
    - api_endpoint_validation
    - performance_testing
    - security_scanning
    - docker_build_validation
  validation:
    input_validation: true
    output_validation: true
    format_validation: true
  performance:
    timeout_seconds: 7200
    retry_attempts: 3
  logging:
    level: INFO
    log_file: rag-deployer.log
  integrations:
    - fastapi
    - qdrant_client
    - docker
    - pytest
  usage_tracking:
    enabled: true
    metrics_file: examples/usage-statistics.md
    log_file: examples/deployment-log.json
```

### 3. Input/Output Models

#### Technical Writer Input Model
```typescript
interface TechnicalWriterInput {
  topic: string;                    // Chapter topic
  module: string;                   // Module number/name
  key_concepts: string[];           // Main concepts to cover
  target_length: number;            // Word count (1500-3000)
  audience_level: 'undergraduate' | 'graduate' | 'professional';
  include_code: boolean;            // Include code examples
  include_diagrams: boolean;        // Include Mermaid diagrams
  related_chapters?: string[];      // Cross-reference links
  prerequisites?: string[];         // Prerequisites for this chapter
  learning_objectives?: string[];   // Learning objectives
  tone?: 'educational_professional' | 'casual_explanatory' | 'formal_academic';
}
```

#### Technical Writer Output Model
```typescript
interface TechnicalWriterOutput {
  content: string;                  // MDX formatted content
  metadata: {
    word_count: number;
    code_examples: number;
    diagrams: number;
    headings: number;
    generation_time: number;        // seconds
  };
  quality_score: number;            // 0-10
  validation_results: {
    syntax_valid: boolean;
    links_valid: boolean;
    heading_hierarchy: boolean;
  };
}
```

#### Code Generator Input Model
```typescript
interface CodeGeneratorInput {
  purpose: string;                  // Purpose of the code
  language: 'python' | 'urdf' | 'xml' | 'yaml'; // Target language
  ros2_distribution?: string;       // Target ROS2 distribution
  include_comments: boolean;        // Include detailed comments
  include_error_handling: boolean;  // Include error handling
  include_tests: boolean;           // Generate tests as well
  complexity_level: 'basic' | 'intermediate' | 'advanced';
  target_framework?: string;        // Specific framework (e.g., rclpy, rclcpp)
}
```

#### Code Generator Output Model
```typescript
interface CodeGeneratorOutput {
  code: string;                     // Generated code
  language: string;                 // Language identifier
  metadata: {
    lines_of_code: number;
    complexity: number;             // Cyclomatic complexity
    generation_time: number;        // seconds
  };
  quality_score: number;            // 0-10
  validation_results: {
    syntax_valid: boolean;
    imports_valid: boolean;
    conventions_followed: boolean;
  };
  suggestions?: string[];           // Improvement suggestions
}
```

#### RAG Specialist Input Model
```typescript
interface RAGSpecialistInput {
  project_name: string;             // Name of the project
  documents_source: string;         // Path to documents
  openai_model: string;             // OpenAI model to use
  qdrant_collection: string;        // Qdrant collection name
  qdrant_url?: string;              // Qdrant server URL
  deployment_target: 'local' | 'docker' | 'cloud';
  api_prefix?: string;              // API prefix (default: /api/v1)
  cors_origins?: string[];          // CORS allowed origins
  max_tokens?: number;              // Max tokens for responses
}
```

#### RAG Specialist Output Model
```typescript
interface RAGSpecialistOutput {
  project_structure: string[];      // List of created files
  api_endpoints: string[];          // List of created endpoints
  metadata: {
    files_created: number;
    endpoints_created: number;
    deployment_time: number;        // seconds
  };
  quality_score: number;            // 0-10
  validation_results: {
    endpoints_working: boolean;
    performance_acceptable: boolean;
    security_checked: boolean;
  };
}
```

### 4. Evidence and Metrics Models

#### Generation Log Model
```json
{
  "generation_id": "string",
  "timestamp": "ISO8601",
  "component": "technical-writer|code-generator|rag-specialist|docusaurus-chapter-creator|ros2-code-validator|rag-deployer",
  "version": "string",
  "input_parameters": {},
  "output_metadata": {
    "size": "number",
    "generation_time_seconds": "number",
    "quality_score": "number"
  },
  "success": "boolean",
  "error_message": "string (if failed)"
}
```

#### Usage Statistics Model
```json
{
  "component": "string",
  "period": {
    "start_date": "ISO8601",
    "end_date": "ISO8601",
    "duration_days": "number"
  },
  "metrics": {
    "total_generations": "number",
    "total_volume": "number (words/lines)",
    "average_generation_time": "number (seconds)",
    "quality_score_average": "number",
    "success_rate_percent": "number",
    "reusability_percent": "number"
  },
  "projects_used_in": ["string"],
  "time_saved_hours": "number",
  "efficiency_gain_percent": "number"
}
```

### 5. Validation Models

#### Content Validation Model
```json
{
  "validation_id": "string",
  "timestamp": "ISO8601",
  "content_type": "chapter|code|backend",
  "file_path": "string",
  "checks": {
    "syntax": {
      "passed": "boolean",
      "details": "string"
    },
    "format": {
      "passed": "boolean",
      "details": "string"
    },
    "links": {
      "passed": "boolean",
      "details": "string"
    },
    "structure": {
      "passed": "boolean",
      "details": "string"
    }
  },
  "overall_score": "number (0-10)",
  "status": "pass|fail|warning"
}
```

## Data Flow

### Subagent Data Flow
```
Input Parameters → Subagent Configuration → AI Model → Validation → Output Generation → Metrics Tracking
```

### Skill Data Flow
```
Input Data → Skill Configuration → Processing Logic → Validation → Output Generation → Metrics Tracking
```

## Quality Assurance Models

### Quality Score Calculation
Quality scores are calculated based on multiple factors:
- Technical accuracy (40%)
- Code quality (25%)
- Format compliance (20%)
- User satisfaction (15%)

### Performance Monitoring
- Response time tracking
- Error rate monitoring
- Resource utilization
- Success rate measurement

## Conclusion

This data model provides the foundation for the Reusable Intelligence System with well-defined structures for configuration, input/output, validation, and metrics tracking. The modular design ensures each component can function independently while maintaining consistency across the system.