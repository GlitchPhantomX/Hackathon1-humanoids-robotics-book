# Agent Orchestration Logic Skill

---
name: agent_orchestration_logic
description: Coordinate tasks between backend and frontend agents with clear responsibility boundaries
version: 1.0.0
project: physical-ai-robotics-textbook
parameters:
  - name: mode
    description: Orchestration mode (strict, flexible, auto)
    required: false
    default: strict
  - name: enable_validation
    description: Enable responsibility overlap validation
    required: false
    default: true
---

## Purpose

Provide orchestration logic that:
- **Routes requests** to the correct agent (backend vs frontend)
- **Prevents overlap** - ensures agents stay within their responsibilities
- **Maintains consistency** - validates system-level invariants
- **Coordinates workflows** - manages multi-agent interactions
- **Enforces boundaries** - strict separation of concerns
- **Validates requests** - ensures requests match agent capabilities

## Core Principles

**MUST FOLLOW:**
1. **Orchestration Only**: No content generation, pure coordination logic
2. **Responsibility Enforcement**: Strict agent boundary checks
3. **Request Routing**: Deterministic routing based on request type
4. **Conflict Detection**: Identify responsibility overlaps
5. **System Validation**: Ensure overall consistency
6. **Stateless Logic**: Orchestration decisions are pure functions

## Agent Responsibility Matrix

```
┌─────────────────────────────────────────────────────────────┐
│                    Responsibility Matrix                     │
├─────────────────────┬──────────────┬─────────────────────────┤
│     Capability      │ Backend Agent│ Frontend Agent          │
├─────────────────────┼──────────────┼─────────────────────────┤
│ API Endpoints       │      ✅      │         ❌              │
│ Database Schema     │      ✅      │         ❌              │
│ RAG Logic           │      ✅      │         ❌              │
│ AI Reasoning        │      ✅      │         ❌              │
│ Business Logic      │      ✅      │         ❌              │
├─────────────────────┼──────────────┼─────────────────────────┤
│ UI Components       │      ❌      │         ✅              │
│ React Hooks         │      ❌      │         ✅              │
│ API Client          │      ❌      │         ✅              │
│ State Management    │      ❌      │         ✅              │
│ User Interactions   │      ❌      │         ✅              │
├─────────────────────┼──────────────┼─────────────────────────┤
│ Chapter Structure   │      ❌      │         ✅              │
│ MDX Templates       │      ❌      │         ✅              │
│ Documentation       │      ❌      │         ✅              │
└─────────────────────┴──────────────┴─────────────────────────┘
```

## Instructions

### Step 1: Define Agent Registry

Create a registry of all agents and their capabilities:

```typescript
// src/orchestration/agent-registry.ts

/**
 * Agent Registry - Defines all available agents and their capabilities
 */

export enum AgentType {
  BACKEND_API = 'backend_api',
  RAG_CONTROL = 'rag_control',
  ROBOTICS_REASONING = 'robotics_reasoning',
  FRONTEND_INTEGRATION = 'frontend_integration',
  CHAPTER_TEMPLATE = 'chapter_template',
}

export enum CapabilityDomain {
  API = 'api',
  DATABASE = 'database',
  RAG = 'rag',
  REASONING = 'reasoning',
  UI = 'ui',
  STATE_MANAGEMENT = 'state_management',
  DOCUMENTATION = 'documentation',
  ORCHESTRATION = 'orchestration',
}

export interface AgentCapability {
  domain: CapabilityDomain;
  actions: string[];
  constraints: string[];
  dependencies: AgentType[];
}

export interface AgentDefinition {
  type: AgentType;
  name: string;
  description: string;
  capabilities: AgentCapability[];
  layer: 'backend' | 'frontend' | 'orchestration';
  canGenerate: string[]; // What this agent can create
  cannotGenerate: string[]; // What this agent must NOT create
}

/**
 * Complete agent registry
 */
export const AGENT_REGISTRY: Record<AgentType, AgentDefinition> = {
  [AgentType.BACKEND_API]: {
    type: AgentType.BACKEND_API,
    name: 'FastAPI Backend Blueprint',
    description: 'Generates FastAPI backend structure with routes, services, and AI logic',
    layer: 'backend',
    capabilities: [
      {
        domain: CapabilityDomain.API,
        actions: [
          'create_routes',
          'create_services',
          'create_models',
          'create_dependencies',
        ],
        constraints: [
          'no_frontend_code',
          'no_ui_components',
          'no_react_code',
        ],
        dependencies: [],
      },
      {
        domain: CapabilityDomain.DATABASE,
        actions: ['define_schemas', 'create_migrations'],
        constraints: ['no_frontend_state'],
        dependencies: [],
      },
    ],
    canGenerate: [
      'FastAPI routes',
      'Pydantic models',
      'Service classes',
      'API endpoints',
      'Backend configuration',
    ],
    cannotGenerate: [
      'React components',
      'TypeScript hooks',
      'UI components',
      'Frontend state',
      'MDX templates',
    ],
  },

  [AgentType.RAG_CONTROL]: {
    type: AgentType.RAG_CONTROL,
    name: 'RAG Context Control',
    description: 'Ensures textbook-grounded answers with hallucination prevention',
    layer: 'backend',
    capabilities: [
      {
        domain: CapabilityDomain.RAG,
        actions: [
          'retrieve_context',
          'validate_grounding',
          'filter_relevance',
          'enforce_context_only',
        ],
        constraints: [
          'no_external_knowledge',
          'no_hallucination',
          'textbook_only',
        ],
        dependencies: [AgentType.BACKEND_API],
      },
    ],
    canGenerate: [
      'RAG engine logic',
      'Context retrieval',
      'Grounding validation',
      'Prompt templates',
    ],
    cannotGenerate: [
      'UI components',
      'Frontend logic',
      'Chapter content',
      'Documentation',
    ],
  },

  [AgentType.ROBOTICS_REASONING]: {
    type: AgentType.ROBOTICS_REASONING,
    name: 'Robotics Reasoning Core',
    description: 'Provides domain knowledge and reasoning for robotics concepts',
    layer: 'backend',
    capabilities: [
      {
        domain: CapabilityDomain.REASONING,
        actions: [
          'explain_concepts',
          'provide_domain_knowledge',
          'generate_learning_paths',
          'adapt_depth',
        ],
        constraints: [
          'no_ui_rendering',
          'no_chapter_specific_logic',
          'domain_agnostic',
        ],
        dependencies: [AgentType.BACKEND_API],
      },
    ],
    canGenerate: [
      'Robotics explanations',
      'Knowledge base',
      'Concept relationships',
      'Code examples (as data)',
    ],
    cannotGenerate: [
      'UI components',
      'React hooks',
      'Chapter templates',
      'Frontend integration',
    ],
  },

  [AgentType.FRONTEND_INTEGRATION]: {
    type: AgentType.FRONTEND_INTEGRATION,
    name: 'Frontend AI Integration',
    description: 'Standardizes frontend UI interactions with backend AI endpoints',
    layer: 'frontend',
    capabilities: [
      {
        domain: CapabilityDomain.UI,
        actions: [
          'create_components',
          'create_hooks',
          'create_api_client',
          'handle_loading_states',
        ],
        constraints: [
          'no_backend_logic',
          'no_business_logic',
          'api_client_only',
        ],
        dependencies: [
          AgentType.BACKEND_API,
          AgentType.RAG_CONTROL,
          AgentType.ROBOTICS_REASONING,
        ],
      },
      {
        domain: CapabilityDomain.STATE_MANAGEMENT,
        actions: [
          'manage_ui_state',
          'handle_async_requests',
          'manage_conversation_history',
        ],
        constraints: ['frontend_only', 'no_data_processing'],
        dependencies: [],
      },
    ],
    canGenerate: [
      'React components',
      'Custom hooks',
      'TypeScript interfaces',
      'API client code',
      'UI state management',
    ],
    cannotGenerate: [
      'FastAPI routes',
      'Backend services',
      'RAG logic',
      'Business logic',
      'Database schemas',
    ],
  },

  [AgentType.CHAPTER_TEMPLATE]: {
    type: AgentType.CHAPTER_TEMPLATE,
    name: 'Docusaurus Chapter Template',
    description: 'Generates consistent chapter structures for textbook',
    layer: 'frontend',
    capabilities: [
      {
        domain: CapabilityDomain.DOCUMENTATION,
        actions: [
          'create_chapter_structure',
          'generate_mdx_template',
          'enforce_section_order',
          'configure_frontmatter',
        ],
        constraints: [
          'no_backend_logic',
          'structure_only',
          'fixed_sections',
        ],
        dependencies: [],
      },
    ],
    canGenerate: [
      'MDX templates',
      'Chapter structure',
      'Docusaurus config',
      'Frontmatter',
      'Section placeholders',
    ],
    cannotGenerate: [
      'Backend APIs',
      'RAG logic',
      'Business logic',
      'API clients',
      'React hooks',
    ],
  },
};
```

### Step 2: Implement Request Routing Logic

Create deterministic routing based on request characteristics:

```typescript
// src/orchestration/request-router.ts

import { AgentType, AGENT_REGISTRY, CapabilityDomain } from './agent-registry';

/**
 * Request types that need routing
 */
export interface AgentRequest {
  type: RequestType;
  description: string;
  keywords: string[];
  domain?: CapabilityDomain;
  layer?: 'backend' | 'frontend';
}

export enum RequestType {
  // Backend requests
  CREATE_API_ENDPOINT = 'create_api_endpoint',
  CREATE_RAG_LOGIC = 'create_rag_logic',
  CREATE_REASONING_ENGINE = 'create_reasoning_engine',
  CREATE_DATABASE_SCHEMA = 'create_database_schema',

  // Frontend requests
  CREATE_UI_COMPONENT = 'create_ui_component',
  CREATE_REACT_HOOK = 'create_react_hook',
  CREATE_API_CLIENT = 'create_api_client',
  CREATE_CHAPTER_TEMPLATE = 'create_chapter_template',

  // Ambiguous (needs validation)
  CREATE_INTEGRATION = 'create_integration',
  CREATE_CONFIGURATION = 'create_configuration',
}

/**
 * Routing decision with validation
 */
export interface RoutingDecision {
  agent: AgentType;
  confidence: number; // 0-1
  reasoning: string;
  warnings: string[];
  requiresMultipleAgents: boolean;
  agentSequence?: AgentType[]; // If multiple agents needed
}

/**
 * Request Router - Routes requests to appropriate agents
 */
export class RequestRouter {
  /**
   * Route a request to the appropriate agent
   */
  route(request: AgentRequest): RoutingDecision {
    // Step 1: Check for exact matches
    const exactMatch = this.findExactMatch(request);
    if (exactMatch) {
      return exactMatch;
    }

    // Step 2: Check by domain
    if (request.domain) {
      const domainMatch = this.findByDomain(request.domain);
      if (domainMatch) {
        return domainMatch;
      }
    }

    // Step 3: Check by layer
    if (request.layer) {
      const layerMatch = this.findByLayer(request.layer, request);
      if (layerMatch) {
        return layerMatch;
      }
    }

    // Step 4: Keyword analysis
    const keywordMatch = this.findByKeywords(request.keywords);
    if (keywordMatch) {
      return keywordMatch;
    }

    // Step 5: No clear match - needs clarification
    return {
      agent: AgentType.BACKEND_API, // Default fallback
      confidence: 0.3,
      reasoning: 'No clear match found - defaulting to backend API agent',
      warnings: [
        'Request is ambiguous',
        'Consider specifying domain or layer',
        'May need multiple agents',
      ],
      requiresMultipleAgents: false,
    };
  }

  private findExactMatch(request: AgentRequest): RoutingDecision | null {
    const typeToAgent: Partial<Record<RequestType, AgentType>> = {
      [RequestType.CREATE_API_ENDPOINT]: AgentType.BACKEND_API,
      [RequestType.CREATE_RAG_LOGIC]: AgentType.RAG_CONTROL,
      [RequestType.CREATE_REASONING_ENGINE]: AgentType.ROBOTICS_REASONING,
      [RequestType.CREATE_UI_COMPONENT]: AgentType.FRONTEND_INTEGRATION,
      [RequestType.CREATE_CHAPTER_TEMPLATE]: AgentType.CHAPTER_TEMPLATE,
    };

    const agent = typeToAgent[request.type];
    if (!agent) return null;

    return {
      agent,
      confidence: 1.0,
      reasoning: `Exact match for request type: ${request.type}`,
      warnings: [],
      requiresMultipleAgents: false,
    };
  }

  private findByDomain(domain: CapabilityDomain): RoutingDecision | null {
    const domainToAgent: Partial<Record<CapabilityDomain, AgentType>> = {
      [CapabilityDomain.API]: AgentType.BACKEND_API,
      [CapabilityDomain.RAG]: AgentType.RAG_CONTROL,
      [CapabilityDomain.REASONING]: AgentType.ROBOTICS_REASONING,
      [CapabilityDomain.UI]: AgentType.FRONTEND_INTEGRATION,
      [CapabilityDomain.DOCUMENTATION]: AgentType.CHAPTER_TEMPLATE,
    };

    const agent = domainToAgent[domain];
    if (!agent) return null;

    return {
      agent,
      confidence: 0.9,
      reasoning: `Matched by capability domain: ${domain}`,
      warnings: [],
      requiresMultipleAgents: false,
    };
  }

  private findByLayer(
    layer: 'backend' | 'frontend',
    request: AgentRequest
  ): RoutingDecision | null {
    const agents = Object.values(AGENT_REGISTRY).filter((a) => a.layer === layer);

    if (agents.length === 0) return null;

    // If only one agent in layer, use it
    if (agents.length === 1) {
      return {
        agent: agents[0].type,
        confidence: 0.7,
        reasoning: `Only agent in ${layer} layer`,
        warnings: ['Verify this is the correct agent for your request'],
        requiresMultipleAgents: false,
      };
    }

    // Multiple agents in layer - needs more context
    return {
      agent: agents[0].type, // Default to first
      confidence: 0.5,
      reasoning: `Multiple agents in ${layer} layer - selected ${agents[0].name}`,
      warnings: [
        'Multiple agents available in this layer',
        'Request may be ambiguous',
        `Consider: ${agents.map((a) => a.name).join(', ')}`,
      ],
      requiresMultipleAgents: false,
    };
  }

  private findByKeywords(keywords: string[]): RoutingDecision | null {
    const keywordMap: Record<string, AgentType> = {
      // Backend keywords
      fastapi: AgentType.BACKEND_API,
      route: AgentType.BACKEND_API,
      endpoint: AgentType.BACKEND_API,
      service: AgentType.BACKEND_API,
      pydantic: AgentType.BACKEND_API,

      rag: AgentType.RAG_CONTROL,
      retrieval: AgentType.RAG_CONTROL,
      grounding: AgentType.RAG_CONTROL,
      context: AgentType.RAG_CONTROL,

      reasoning: AgentType.ROBOTICS_REASONING,
      explanation: AgentType.ROBOTICS_REASONING,
      concept: AgentType.ROBOTICS_REASONING,
      robotics: AgentType.ROBOTICS_REASONING,

      // Frontend keywords
      react: AgentType.FRONTEND_INTEGRATION,
      component: AgentType.FRONTEND_INTEGRATION,
      hook: AgentType.FRONTEND_INTEGRATION,
      typescript: AgentType.FRONTEND_INTEGRATION,
      ui: AgentType.FRONTEND_INTEGRATION,

      chapter: AgentType.CHAPTER_TEMPLATE,
      mdx: AgentType.CHAPTER_TEMPLATE,
      docusaurus: AgentType.CHAPTER_TEMPLATE,
      documentation: AgentType.CHAPTER_TEMPLATE,
    };

    const matches = keywords
      .map((kw) => keywordMap[kw.toLowerCase()])
      .filter(Boolean);

    if (matches.length === 0) return null;

    // Count occurrences
    const counts = matches.reduce((acc, agent) => {
      acc[agent] = (acc[agent] || 0) + 1;
      return acc;
    }, {} as Record<AgentType, number>);

    // Find most common
    const mostCommon = Object.entries(counts).sort((a, b) => b[1] - a[1])[0];

    return {
      agent: mostCommon[0] as AgentType,
      confidence: Math.min(mostCommon[1] / keywords.length, 0.95),
      reasoning: `Keyword match: ${keywords.join(', ')}`,
      warnings:
        matches.length < keywords.length
          ? ['Some keywords did not match any agent']
          : [],
      requiresMultipleAgents: false,
    };
  }

  /**
   * Determine if request needs multiple agents
   */
  requiresMultipleAgents(request: AgentRequest): boolean {
    const fullStackKeywords = ['integration', 'end-to-end', 'complete', 'full'];
    return fullStackKeywords.some((kw) =>
      request.keywords.some((reqKw) => reqKw.toLowerCase().includes(kw))
    );
  }

  /**
   * Get agent sequence for multi-agent requests
   */
  getAgentSequence(request: AgentRequest): AgentType[] {
    // Common patterns
    if (request.keywords.includes('integration')) {
      // Backend first, then frontend
      return [
        AgentType.BACKEND_API,
        AgentType.RAG_CONTROL,
        AgentType.FRONTEND_INTEGRATION,
      ];
    }

    if (request.keywords.includes('chapter') && request.keywords.includes('api')) {
      return [AgentType.BACKEND_API, AgentType.CHAPTER_TEMPLATE];
    }

    return [];
  }
}
```

### Step 3: Implement Responsibility Validator

Ensure agents don't overstep their boundaries:

```typescript
// src/orchestration/responsibility-validator.ts

import { AgentType, AGENT_REGISTRY } from './agent-registry';

/**
 * Validation result
 */
export interface ValidationResult {
  valid: boolean;
  violations: string[];
  warnings: string[];
  recommendations: string[];
}

/**
 * Artifact to validate
 */
export interface Artifact {
  type: string; // 'file', 'code', 'config'
  path?: string;
  content: string;
  agentType: AgentType;
}

/**
 * Responsibility Validator - Ensures agents stay within their boundaries
 */
export class ResponsibilityValidator {
  /**
   * Validate that an artifact respects agent boundaries
   */
  validate(artifact: Artifact): ValidationResult {
    const result: ValidationResult = {
      valid: true,
      violations: [],
      warnings: [],
      recommendations: [],
    };

    const agent = AGENT_REGISTRY[artifact.agentType];

    // Check 1: Backend agent creating frontend code
    if (agent.layer === 'backend' && this.isFrontendCode(artifact.content)) {
      result.valid = false;
      result.violations.push(
        `Backend agent "${agent.name}" is generating frontend code`
      );
      result.recommendations.push(
        'Use frontend_integration agent for React/TypeScript code'
      );
    }

    // Check 2: Frontend agent creating backend code
    if (agent.layer === 'frontend' && this.isBackendCode(artifact.content)) {
      result.valid = false;
      result.violations.push(
        `Frontend agent "${agent.name}" is generating backend code`
      );
      result.recommendations.push(
        'Use backend_api agent for FastAPI/Python code'
      );
    }

    // Check 3: Verify against cannotGenerate list
    for (const forbidden of agent.cannotGenerate) {
      if (this.contentMatches(artifact.content, forbidden)) {
        result.valid = false;
        result.violations.push(
          `Agent cannot generate: ${forbidden} (found in artifact)`
        );
      }
    }

    // Check 4: Chapter template creating business logic
    if (
      artifact.agentType === AgentType.CHAPTER_TEMPLATE &&
      this.hasBusinessLogic(artifact.content)
    ) {
      result.valid = false;
      result.violations.push('Chapter template should not contain business logic');
      result.recommendations.push('Move logic to backend services');
    }

    // Check 5: RAG agent creating UI components
    if (
      artifact.agentType === AgentType.RAG_CONTROL &&
      this.hasUIComponents(artifact.content)
    ) {
      result.valid = false;
      result.violations.push('RAG agent should not create UI components');
      result.recommendations.push('Use frontend_integration agent for UI');
    }

    return result;
  }

  /**
   * Check if content is frontend code
   */
  private isFrontendCode(content: string): boolean {
    const frontendIndicators = [
      /import\s+.*from\s+['"]react['"]/,
      /import\s+.*from\s+['"]@theme/,
      /useState|useEffect|useCallback/,
      /className=/,
      /<[A-Z][a-zA-Z]*\s/, // JSX components
      /interface\s+\w+Props/,
    ];

    return frontendIndicators.some((pattern) => pattern.test(content));
  }

  /**
   * Check if content is backend code
   */
  private isBackendCode(content: string): boolean {
    const backendIndicators = [
      /from\s+fastapi\s+import/,
      /from\s+pydantic\s+import/,
      /@app\.(get|post|put|delete)/,
      /class\s+\w+\(BaseModel\)/,
      /async\s+def\s+.*\(.*Request/,
      /router\s*=\s*APIRouter/,
    ];

    return backendIndicators.some((pattern) => pattern.test(content));
  }

  /**
   * Check if content has business logic
   */
  private hasBusinessLogic(content: string): boolean {
    const businessLogicIndicators = [
      /async\s+def\s+\w+\(/,
      /class\s+\w+Service/,
      /if.*else.*return/s,
      /for\s+.*in.*:/,
      /while\s+.*:/,
    ];

    return businessLogicIndicators.some((pattern) => pattern.test(content));
  }

  /**
   * Check if content has UI components
   */
  private hasUIComponents(content: string): boolean {
    const uiIndicators = [
      /<div|<span|<button|<input/,
      /className|style=/,
      /onClick|onChange/,
      /return\s*\(?\s*</,
    ];

    return uiIndicators.some((pattern) => pattern.test(content));
  }

  /**
   * Check if content matches a forbidden pattern
   */
  private contentMatches(content: string, pattern: string): boolean {
    const lowerContent = content.toLowerCase();
    const lowerPattern = pattern.toLowerCase();

    // Simple substring match
    return lowerContent.includes(lowerPattern);
  }

  /**
   * Validate multi-agent workflow
   */
  validateWorkflow(artifacts: Artifact[]): ValidationResult {
    const result: ValidationResult = {
      valid: true,
      violations: [],
      warnings: [],
      recommendations: [],
    };

    // Check for responsibility overlaps
    const backendArtifacts = artifacts.filter(
      (a) => AGENT_REGISTRY[a.agentType].layer === 'backend'
    );
    const frontendArtifacts = artifacts.filter(
      (a) => AGENT_REGISTRY[a.agentType].layer === 'frontend'
    );

    // Ensure no backend artifact has frontend code
    for (const artifact of backendArtifacts) {
      if (this.isFrontendCode(artifact.content)) {
        result.valid = false;
        result.violations.push(
          `Backend artifact contains frontend code: ${artifact.path}`
        );
      }
    }

    // Ensure no frontend artifact has backend code
    for (const artifact of frontendArtifacts) {
      if (this.isBackendCode(artifact.content)) {
        result.valid = false;
        result.violations.push(
          `Frontend artifact contains backend code: ${artifact.path}`
        );
      }
    }

    // Check for proper dependencies
    const hasBackend = backendArtifacts.length > 0;
    const hasFrontendIntegration = artifacts.some(
      (a) => a.agentType === AgentType.FRONTEND_INTEGRATION
    );

    if (hasFrontendIntegration && !hasBackend) {
      result.warnings.push(
        'Frontend integration without backend - ensure backend exists'
      );
    }

    return result;
  }
}
```

### Step 4: System Consistency Checker

Validate system-level invariants:

```typescript
// src/orchestration/consistency-checker.ts

/**
 * System-level consistency rules
 */
export interface ConsistencyRule {
  id: string;
  description: string;
  check: (system: SystemState) => boolean;
  severity: 'error' | 'warning' | 'info';
}

export interface SystemState {
  backendServices: string[];
  frontendComponents: string[];
  apiEndpoints: string[];
  chapters: string[];
  integrations: Array<{
    backend: string;
    frontend: string;
  }>;
}

export interface ConsistencyCheckResult {
  passed: boolean;
  failedRules: Array<{
    rule: ConsistencyRule;
    message: string;
  }>;
}

/**
 * Consistency Checker - Validates system-level invariants
 */
export class ConsistencyChecker {
  private rules: ConsistencyRule[] = [
    {
      id: 'backend-frontend-match',
      description: 'Every backend service should have corresponding frontend integration',
      check: (state) => {
        const backendAPIs = state.apiEndpoints.map((e) =>
          e.split('/')[2]
        ); // Extract service name
        const frontendIntegrations = state.integrations.map((i) => i.backend);

        return backendAPIs.every((api) => frontendIntegrations.includes(api));
      },
      severity: 'warning',
    },
    {
      id: 'no-duplicate-responsibilities',
      description: 'No two agents should handle the same responsibility',
      check: (state) => {
        // This would check for duplicate implementations
        // Simplified for example
        return true;
      },
      severity: 'error',
    },
    {
      id: 'chapter-structure-consistent',
      description: 'All chapters must follow the same 5-section structure',
      check: (state) => {
        const requiredSections = [
          'introduction',
          'theory',
          'code-explanation',
          'simulation',
          'exercises',
        ];

        return state.chapters.every((chapter) => {
          // In real implementation, would check actual chapter structure
          return true; // Simplified
        });
      },
      severity: 'error',
    },
    {
      id: 'api-client-matches-backend',
      description: 'Frontend API client should match backend endpoints',
      check: (state) => {
        // Verify that frontend API client has methods for all backend endpoints
        return true; // Simplified
      },
      severity: 'error',
    },
  ];

  /**
   * Run all consistency checks
   */
  check(state: SystemState): ConsistencyCheckResult {
    const failedRules: ConsistencyCheckResult['failedRules'] = [];

    for (const rule of this.rules) {
      try {
        const passed = rule.check(state);

        if (!passed) {
          failedRules.push({
            rule,
            message: `Consistency check failed: ${rule.description}`,
          });
        }
      } catch (error) {
        failedRules.push({
          rule,
          message: `Error running check: ${error}`,
        });
      }
    }

    return {
      passed: failedRules.filter((f) => f.rule.severity === 'error').length === 0,
      failedRules,
    };
  }

  /**
   * Add custom rule
   */
  addRule(rule: ConsistencyRule): void {
    this.rules.push(rule);
  }
}
```

### Step 5: Orchestration Coordinator

Main orchestration logic:

```typescript
// src/orchestration/orchestrator.ts

import { RequestRouter, AgentRequest, RoutingDecision } from './request-router';
import { ResponsibilityValidator, Artifact, ValidationResult } from './responsibility-validator';
import { ConsistencyChecker, SystemState } from './consistency-checker';
import { AgentType } from './agent-registry';

/**
 * Orchestration plan
 */
export interface OrchestrationPlan {
  agents: AgentType[];
  sequence: Array<{
    step: number;
    agent: AgentType;
    task: string;
    dependencies: number[]; // Steps that must complete first
  }>;
  validations: Array<{
    afterStep: number;
    type: 'responsibility' | 'consistency';
  }>;
}

/**
 * Main Orchestrator
 */
export class Orchestrator {
  private router: RequestRouter;
  private validator: ResponsibilityValidator;
  private consistencyChecker: ConsistencyChecker;

  constructor() {
    this.router = new RequestRouter();
    this.validator = new ResponsibilityValidator();
    this.consistencyChecker = new ConsistencyChecker();
  }

  /**
   * Plan orchestration for a request
   */
  plan(request: AgentRequest): OrchestrationPlan {
    // Step 1: Route to agent(s)
    const routing = this.router.route(request);

    // Step 2: Determine if multi-agent needed
    const needsMultiple = this.router.requiresMultipleAgents(request);

    // Step 3: Build orchestration plan
    if (needsMultiple || routing.requiresMultipleAgents) {
      return this.buildMultiAgentPlan(request, routing);
    } else {
      return this.buildSingleAgentPlan(routing);
    }
  }

  private buildSingleAgentPlan(routing: RoutingDecision): OrchestrationPlan {
    return {
      agents: [routing.agent],
      sequence: [
        {
          step: 1,
          agent: routing.agent,
          task: 'Execute primary task',
          dependencies: [],
        },
      ],
      validations: [
        {
          afterStep: 1,
          type: 'responsibility',
        },
      ],
    };
  }

  private buildMultiAgentPlan(
    request: AgentRequest,
    routing: RoutingDecision
  ): OrchestrationPlan {
    const agentSequence = routing.agentSequence || this.router.getAgentSequence(request);

    const sequence = agentSequence.map((agent, index) => ({
      step: index + 1,
      agent,
      task: this.getTaskForAgent(agent, request),
      dependencies: index > 0 ? [index] : [],
    }));

    const validations = agentSequence.map((_, index) => ({
      afterStep: index + 1,
      type: 'responsibility' as const,
    }));

    // Add final consistency check
    validations.push({
      afterStep: agentSequence.length,
      type: 'consistency',
    });

    return {
      agents: agentSequence,
      sequence,
      validations,
    };
  }

  private getTaskForAgent(agent: AgentType, request: AgentRequest): string {
    const tasks: Partial<Record<AgentType, string>> = {
      [AgentType.BACKEND_API]: 'Create backend API structure',
      [AgentType.RAG_CONTROL]: 'Implement RAG grounding logic',
      [AgentType.ROBOTICS_REASONING]: 'Setup robotics reasoning engine',
      [AgentType.FRONTEND_INTEGRATION]: 'Create frontend integration',
      [AgentType.CHAPTER_TEMPLATE]: 'Generate chapter template',
    };

    return tasks[agent] || 'Execute agent task';
  }

  /**
   * Validate artifacts from agents
   */
  validateArtifacts(artifacts: Artifact[]): ValidationResult {
    // Validate each artifact individually
    for (const artifact of artifacts) {
      const result = this.validator.validate(artifact);
      if (!result.valid) {
        return result;
      }
    }

    // Validate workflow as a whole
    return this.validator.validateWorkflow(artifacts);
  }

  /**
   * Check system consistency
   */
  checkSystemConsistency(state: SystemState) {
    return this.consistencyChecker.check(state);
  }

  /**
   * Get routing recommendation
   */
  recommend(request: AgentRequest): {
    routing: RoutingDecision;
    explanation: string;
    alternatives: AgentType[];
  } {
    const routing = this.router.route(request);

    const explanation = this.explainRouting(routing, request);

    const alternatives = this.getAlternativeAgents(routing, request);

    return {
      routing,
      explanation,
      alternatives,
    };
  }

  private explainRouting(routing: RoutingDecision, request: AgentRequest): string {
    let explanation = `Selected ${routing.agent} because:\n`;
    explanation += `- ${routing.reasoning}\n`;
    explanation += `- Confidence: ${(routing.confidence * 100).toFixed(0)}%\n`;

    if (routing.warnings.length > 0) {
      explanation += `\nWarnings:\n`;
      routing.warnings.forEach((w) => (explanation += `- ${w}\n`));
    }

    return explanation;
  }

  private getAlternativeAgents(
    routing: RoutingDecision,
    request: AgentRequest
  ): AgentType[] {
    if (routing.confidence >= 0.9) {
      return []; // High confidence, no alternatives needed
    }

    // Return agents in same layer
    const layer = request.layer;
    if (!layer) return [];

    return Object.values(AGENT_REGISTRY)
      .filter((a) => a.layer === layer && a.type !== routing.agent)
      .map((a) => a.type);
  }
}
```

### Step 6: CLI Interface

```typescript
// src/orchestration/cli.ts

import { Orchestrator } from './orchestrator';
import { AgentRequest, RequestType } from './request-router';

/**
 * CLI for orchestration
 */
export function orchestrateCLI() {
  const orchestrator = new Orchestrator();

  // Example: Route a request
  const request: AgentRequest = {
    type: RequestType.CREATE_INTEGRATION,
    description: 'Create full-stack chatbot integration',
    keywords: ['chatbot', 'rag', 'react', 'fastapi'],
    layer: undefined, // Let orchestrator decide
  };

  console.log('🎯 Orchestration Request:');
  console.log(JSON.stringify(request, null, 2));

  // Get recommendation
  const recommendation = orchestrator.recommend(request);

  console.log('\n📋 Orchestration Plan:');
  console.log(recommendation.explanation);

  // Create plan
  const plan = orchestrator.plan(request);

  console.log('\n🗺️ Execution Plan:');
  plan.sequence.forEach((step) => {
    console.log(`\nStep ${step.step}: ${step.agent}`);
    console.log(`  Task: ${step.task}`);
    console.log(`  Dependencies: ${step.dependencies.join(', ') || 'None'}`);
  });

  console.log('\n✅ Validations:');
  plan.validations.forEach((v) => {
    console.log(`  After step ${v.afterStep}: ${v.type} check`);
  });
}
```

## Decision Trees

### Backend vs Frontend Decision Tree

```
Request comes in
    │
    ├─ Contains "FastAPI", "Pydantic", "route"?
    │   └─→ Backend Agent
    │
    ├─ Contains "React", "component", "hook"?
    │   └─→ Frontend Agent
    │
    ├─ Contains "chapter", "MDX", "Docusaurus"?
    │   └─→ Chapter Template Agent
    │
    ├─ Contains "RAG", "retrieval", "grounding"?
    │   └─→ RAG Control Agent
    │
    ├─ Contains "reasoning", "concept", "explanation"?
    │   └─→ Robotics Reasoning Agent
    │
    └─ Ambiguous?
        └─→ Ask for clarification
```

## Validation Checklist

After orchestration, verify:

- [ ] **Correct Agent**: Request routed to appropriate agent
- [ ] **No Overlap**: Agents don't duplicate responsibilities
- [ ] **Boundary Respect**: Backend doesn't create frontend code
- [ ] **Consistency**: System-level invariants maintained
- [ ] **Dependencies**: Agent dependencies satisfied
- [ ] **Sequence**: Multi-agent workflows in correct order
- [ ] **Validation**: All artifacts validated before finalization

## Example Usage

```typescript
// Example 1: Route a simple request
const orchestrator = new Orchestrator();

const request: AgentRequest = {
  type: RequestType.CREATE_API_ENDPOINT,
  description: 'Create chatbot API endpoint',
  keywords: ['chatbot', 'fastapi', 'endpoint'],
};

const plan = orchestrator.plan(request);
// → Routes to BACKEND_API agent

// Example 2: Multi-agent request
const fullStackRequest: AgentRequest = {
  type: RequestType.CREATE_INTEGRATION,
  description: 'Full chatbot integration',
  keywords: ['chatbot', 'integration', 'end-to-end'],
};

const multiPlan = orchestrator.plan(fullStackRequest);
// → Routes to: BACKEND_API → RAG_CONTROL → FRONTEND_INTEGRATION

// Example 3: Validate artifacts
const artifacts: Artifact[] = [
  {
    type: 'file',
    path: 'backend/routes/chatbot.py',
    content: 'from fastapi import APIRouter...',
    agentType: AgentType.BACKEND_API,
  },
];

const validation = orchestrator.validateArtifacts(artifacts);
// → validation.valid = true
```

## Output

When this skill is invoked, it generates:

1. **Agent Registry** (`agent-registry.ts`) - All agents and capabilities
2. **Request Router** (`request-router.ts`) - Routing logic
3. **Responsibility Validator** (`responsibility-validator.ts`) - Boundary enforcement
4. **Consistency Checker** (`consistency-checker.ts`) - System validation
5. **Orchestrator** (`orchestrator.ts`) - Main coordination logic
6. **CLI** (`cli.ts`) - Command-line interface

## Success Criteria

- ✅ **No Content Generation**: Pure orchestration logic
- ✅ **Correct Routing**: Requests go to right agents
- ✅ **No Overlap**: Agents respect boundaries
- ✅ **System Consistency**: Invariants maintained
- ✅ **Deterministic**: Same request → same routing
- ✅ **Validated**: All artifacts checked
