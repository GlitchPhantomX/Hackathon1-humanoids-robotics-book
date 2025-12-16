// @ts-check

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Humanoid Robotics',
  tagline: 'Learning Robotics with Physical AI',
  favicon: 'img/favicon.ico',

  url: 'https://hackathon1-humanoids-robotics-book-lygshd3nv.vercel.app',
  baseUrl: '/',

  organizationName: 'GlitchPhantomX',
  projectName: 'Hackathon1-humanoids-robotics-book',

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