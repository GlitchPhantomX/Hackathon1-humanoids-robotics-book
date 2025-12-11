# Physical AI & Humanoid Robotics Textbook

This project contains a comprehensive textbook on Physical AI and Humanoid Robotics, built with Docusaurus. The textbook covers fundamental concepts, ROS 2 architecture, simulation with Gazebo, NVIDIA Isaac Sim, Vision-Language-Action systems, and capstone projects.

## Table of Contents

The textbook is organized into 6 main modules:
- **Module 0**: Introduction to Physical AI and Humanoid Robotics
- **Module 1**: ROS 2 Architecture & Concepts
- **Module 2**: Simulation with Gazebo
- **Module 3**: NVIDIA Isaac Sim
- **Module 4**: Vision-Language-Action (VLA) Systems
- **Module 5**: Capstone Project

## Features

- Interactive learning with reading time indicators
- Dual-view functionality (Full Lesson/Summary)
- Hands-on exercises with success criteria
- Comprehensive troubleshooting sections
- Code examples in Python, Bash, XML/URDF
- Mermaid diagrams for visual explanations
- Responsive design for all devices

## Getting Started

### Prerequisites
- Node.js (v16.14 or higher)
- npm or yarn
- Git

### Installation
1. Clone the repository
2. Navigate to the docusaurus directory: `cd physical-ai-robotics-textbook/docusaurus`
3. Install dependencies: `npm install`
4. Start the development server: `npm start`

### Development
- Local development server: `npm start`
- Build for production: `npm run build`
- Serve production build locally: `npm run serve`

## Deployment

### GitHub Pages
The site can be deployed to GitHub Pages with the following configuration:

1. Update `docusaurus.config.mjs` with your repository details:
   - Set `url` to your GitHub Pages URL
   - Set `baseUrl` to your project name
   - Update `organizationName` and `projectName`

2. Build the site: `npm run build`

3. The build output will be in the `build` directory

### Other Platforms
The site can also be deployed to:
- Netlify
- Vercel
- AWS S3
- Any static hosting service

## Contributing

To contribute to the textbook:
1. Fork the repository
2. Create a new branch for your changes
3. Make your changes following the existing content structure
4. Test your changes locally
5. Submit a pull request

## Build and Deployment Instructions

### Environment Variables
No special environment variables are required for basic operation.

### Deployment Configuration
- The site uses `@docusaurus/preset-classic`
- Custom CSS is located in `src/css/custom.css` and `static/css/custom.css`
- Components are in `src/components/`
- Documentation is in `docs/`

### GitHub Actions (Optional)
To automate deployment with GitHub Actions, create a workflow file in `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    - uses: actions/setup-node@v3
      with:
        node-version: 18
    - name: Install dependencies
      run: npm install
    - name: Build website
      run: npm run build
    - name: Deploy to GitHub Pages
      uses: peaceiris/actions-gh-pages@v3
      with:
        github_token: ${{ secrets.GITHUB_TOKEN }}
        publish_dir: ./build
```

## Support

For issues with the textbook content or technical problems, please open an issue in the repository.