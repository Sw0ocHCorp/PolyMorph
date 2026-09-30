import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This file runs in Node.js: no client-side code here.

const config: Config = {
  title: 'PolyMorph',
  tagline: 'Architecture, mathematics and mechanisms of the PolyMorph generic robotics control stack',

  // Served locally only for now; set url / baseUrl when the site gets deployed.
  url: 'http://localhost',
  baseUrl: '/',

  organizationName: 'Sw0ocHCorp',
  projectName: 'PolyMorph',

  // A broken link or anchor fails the build instead of shipping a dead page.
  onBrokenLinks: 'throw',
  onBrokenAnchors: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    // `.mdx` pages are MDX (JSX allowed: diagrams, interactive components);
    // `.md` files (the synced CHANGELOG) are plain CommonMark, so a stray `<` or `{`
    // written in a changelog entry can never break the build.
    format: 'detect',
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'throw',
      onBrokenMarkdownImages: 'throw',
    },
  },

  themes: [
    '@docusaurus/theme-mermaid',
    [
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        docsRouteBasePath: '/',
        indexBlog: false,
        indexPages: true,
        highlightSearchTermsOnTargetPage: true,
      },
    ],
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          // The documentation IS the site: pages are served from the root.
          routeBasePath: '/',
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/Sw0ocHCorp/PolyMorph/tree/Total_Rework/docs/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
        svgr: {
          svgrConfig: {
            svgoConfig: {
              plugins: [
                {
                  name: 'preset-default',
                  params: {
                    overrides: {
                      removeTitle: false,
                      removeViewBox: false,
                      // Keep the marker ids exactly as written. svgo's default renames them
                      // to `a`, `b`, … in every file: two diagrams on the same page would
                      // then share `id="a"` and draw each other's arrow heads.
                      cleanupIds: false,
                    },
                  },
                },
              ],
            },
          },
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'PolyMorph',
      items: [
        {type: 'docSidebar', sidebarId: 'docs', position: 'left', label: 'Documentation'},
        {to: '/changelog', label: 'Changelog', position: 'left'},
        {href: 'https://github.com/Sw0ocHCorp/PolyMorph', label: 'GitHub', position: 'right'},
      ],
    },
    footer: {
      style: 'dark',
      copyright: `PolyMorph — generic robotics control stack. Built with Docusaurus.`,
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 3,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['rust', 'toml', 'bash'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
