# Quickstart: Physical AI & Humanoid Robotics Textbook Development

## Prerequisites

- Node.js LTS (v18 or higher)
- npm or yarn package manager
- Git version control
- Text editor or IDE with JavaScript/Markdown support

## Setup Instructions

### 1. Clone the Repository
```bash
git clone [repository-url]
cd physical-ai-robotics-textbook
```

### 2. Navigate to Docusaurus Directory
```bash
cd docusaurus
```

### 3. Install Dependencies
```bash
npm install
# or
yarn install
```

### 4. Start Development Server
```bash
npm run start
# or
yarn start
```

The textbook will be available at `http://localhost:3000`

## Project Structure Overview

```
docusaurus/
├── docs/                    # All textbook content
│   ├── 00-introduction/     # Module 0: Introduction
│   ├── 01-ros2/            # Module 1: ROS 2
│   ├── 02-simulation/      # Module 2: Simulation
│   ├── 03-isaac/           # Module 3: Isaac
│   ├── 04-vla/             # Module 4: VLA
│   └── 05-capstone/        # Module 5: Capstone
├── src/
│   └── components/         # Custom React components
│       ├── ReadingTime.js  # Reading time indicator
│       └── ViewToggle.js   # Full/Summary view toggle
├── static/css/             # Custom CSS styles
└── docusaurus.config.mjs   # Docusaurus configuration
```

## Adding New Content

### Create a New Chapter
1. Navigate to the appropriate module directory
2. Create a new `.md` file with numeric prefix (e.g., `07-new-topic.md`)
3. Follow the standard chapter template from the specification
4. Add the new file to the `_category_.json` if needed

### Create Custom Components
1. Add new component to `src/components/`
2. Use React with proper props validation
3. Import and use in Markdown files with `import` statements

## Building for Production

```bash
npm run build
# or
yarn build
```

The built site will be in the `build/` directory, ready for deployment.

## Testing

### Component Tests
```bash
npm test
# or
yarn test
```

### E2E Tests
```bash
npm run test:e2e
# or
yarn run test:e2e
```

## Deployment

The site is configured for GitHub Pages deployment. After building:

1. The `build/` directory contains the static site
2. Configure your hosting platform to serve from this directory
3. Ensure proper base URL configuration in `docusaurus.config.mjs`

## Common Commands

| Command | Description |
|---------|-------------|
| `npm start` | Start development server |
| `npm run build` | Build for production |
| `npm run serve` | Serve built site locally |
| `npm run swizzle` | Customize Docusaurus theme components |
| `npm run clear` | Clear Docusaurus cache |

## Troubleshooting

### Development Server Won't Start
- Ensure Node.js and npm are properly installed
- Run `npm install` to reinstall dependencies
- Check for port conflicts (default: 3000)

### Build Fails
- Verify all Markdown files follow the specification
- Check for syntax errors in custom components
- Ensure all required frontmatter is present

### Custom Components Not Working
- Verify import statements in Markdown files
- Check component file paths and names
- Ensure React syntax is correct