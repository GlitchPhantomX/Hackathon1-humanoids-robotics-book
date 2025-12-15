// @ts-check

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Humanoid Robotics',
  tagline: 'Learning Robotics with Physical AI',
  favicon: 'img/favicon.ico',

  // ✅ VERCEL KE LIYE - root path use karo
  url: 'https://your-vercel-domain.vercel.app',  // Vercel URL dalo
  baseUrl: '/',  // ✅ Changed from /physical-ai-robotics-textbook/ to /

  organizationName: 'your-organization',
  projectName: 'physical-ai-robotics-textbook',

  onBrokenLinks: 'throw',
  
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  customFields: {
    translationEnabled: false,
    requiresAuth: true,
    apiUrl: process.env.API_URL || 'http://localhost:8000',
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/GlitchPhantomX/Hackathon1-humanoids-robotics-book/edit/main/',
        },
        blog: false,
        theme: {
          customCss: ['./src/css/custom.css', './static/css/custom.css'],
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    navbar: {
      title: 'Humanoid Robotics',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
      ],
    },

    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },

    prism: {
      additionalLanguages: ['bash', 'python', 'javascript', 'typescript'],
    },
  },
};

export default config;