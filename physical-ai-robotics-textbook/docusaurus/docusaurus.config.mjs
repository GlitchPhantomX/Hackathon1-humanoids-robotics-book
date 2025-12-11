// @ts-check

import lightCodeTheme from 'prism-react-renderer/themes/github';
import darkCodeTheme from 'prism-react-renderer/themes/dracula';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Humanoid Robotics',
  tagline: 'Learning Robotics with Physical AI',
  favicon: 'img/favicon.ico',

  url: 'https://your-organization.github.io',  // Change to your actual domain
  baseUrl: '/physical-ai-robotics-textbook/',  // Add trailing slash for production

  organizationName: 'your-organization',  // Change to your GitHub organization
  projectName: 'physical-ai-robotics-textbook',  // Change to your repository name

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // ❌ Removed languages
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/your-organization/physical-ai-robotics-textbook/edit/main/',
        },
        blog: false,
        theme: {
          customCss: [
            require.resolve('./src/css/custom.css'),
            require.resolve('./static/css/custom.css'),
          ],
        },
      },
    ],
  ],

  customFields: {
    apiUrl: process.env.API_URL || 'http://localhost:8000',
  },

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

        // ❌ Removed GitHub link
        // ❌ Removed Language Dropdown
      ],
    },

    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },

    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
  },

  plugins: [
    function () {
      return {
        name: 'custom-webpack-config',
        configureWebpack() {
          return {
            devServer: {
              proxy: [
                {
                  context: ['/api'],
                  target: 'http://localhost:8000',
                  changeOrigin: true,
                  secure: false,
                  logLevel: 'debug',
                },
              ],
            },
          };
        },
      };
    },
  ],
};

export default config;
