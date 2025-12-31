# Research Findings: Multi-Language Translation System

## Research Session: 2025-12-30

### Task 1: Docusaurus i18n Plugin Version Compatibility

**Decision**: Use Docusaurus v3.x with @docusaurus/plugin-content-docs for i18n support
**Rationale**: The project already uses Docusaurus v3.0+ based on the package.json in the specification. The built-in i18n plugin is well-supported and integrates seamlessly with the existing architecture.
**Alternatives considered**:
- Custom translation solution: More complex and error-prone
- Third-party i18n libraries: Would require significant integration work

**Findings**:
- Docusaurus has built-in i18n support via the i18n configuration in docusaurus.config.mjs
- The @docusaurus/plugin-content-docs plugin handles locale-based content routing
- Version 3.x has stable i18n APIs with good documentation and community support

### Task 2: Custom Font Loading in Docusaurus for Urdu/Hindi

**Decision**: Use @font-face CSS declarations with preloaded font files
**Rationale**: This approach provides reliable font loading with good performance characteristics and works well with Docusaurus's static asset handling.
**Alternatives considered**:
- Google Fonts CDN: Potential availability issues in certain regions
- Dynamic font loading: Increased complexity and potential FOUC (Flash of Unstyled Content)

**Findings**:
- Noto Nastaliq Urdu supports Arabic/Persian script used in Urdu
- Noto Sans Devanagari is ideal for Hindi text rendering
- Using font-display: swap ensures text remains visible during font loading
- Font files should be placed in static/fonts/ directory for proper serving

### Task 3: Build Performance Impact of Multi-Language Support

**Decision**: Implement build optimization with locale-specific builds
**Rationale**: While multi-language support increases build complexity, Docusaurus handles this efficiently with its plugin architecture.
**Alternatives considered**:
- Single universal build with all languages: Would create unnecessarily large bundles
- Separate deployments per language: Would complicate maintenance

**Findings**:
- Docusaurus supports building specific locales with `npm run build -- --locale ur`
- Build times will increase proportionally with number of locales but remain manageable
- Static generation per locale ensures optimal runtime performance
- Compression techniques can mitigate bundle size increases

### Task 4: RTL CSS Compatibility with Existing Components

**Decision**: Use attribute-based RTL styling with [dir='rtl'] selectors
**Rationale**: This approach provides precise RTL control while maintaining compatibility with existing LTR styles.
**Alternatives considered**:
- Separate RTL CSS files: Would duplicate much of the existing CSS
- JavaScript-based RTL: Would add runtime overhead and complexity

**Findings**:
- Docusaurus supports RTL via the direction property in locale configuration
- [dir='rtl'] attribute selectors work reliably across browsers
- Most existing components will need minimal adjustments for RTL compatibility
- Flexbox layouts can be reversed with direction: rtl property

## Resolved Unknowns

All previously identified unknowns have been researched and resolved:

- ✅ Docusaurus i18n plugin version compatibility: Confirmed v3.x support
- ✅ Custom font loading approach: Confirmed @font-face method with static assets
- ✅ Build performance impact: Assessed as manageable with proper optimization
- ✅ RTL CSS compatibility: Confirmed attribute-based approach works with existing components

## Next Steps

With research complete, we can proceed to Phase 1: Design & Architecture with confidence that the technical approaches are sound and compatible with the existing system.