---
sidebar_position: 5
---

# Chapter 4: Theming, Search, and Plugins

## Learning Goals
- Understand theming basics for Docusaurus sites
- Learn about local search configuration and optimization
- Identify essential plugins for technical books

## Theming Fundamentals

### Default Theme Components

Docusaurus comes with a default theme that provides all necessary components for documentation sites. The theme includes:

- Navigation header with responsive mobile menu
- Sidebar with collapsible categories
- Breadcrumb navigation
- Search functionality
- Dark/light mode toggle
- Table of contents
- Edit this page links
- Previous/next navigation

### Customizing Colors with CSS Variables

You can customize the theme by overriding CSS variables in your custom CSS file:

```css
/* src/css/custom.css */
:root {
  /* Primary colors */
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: #21af90;
  --ifm-color-primary-darker: #1fa588;
  --ifm-color-primary-darkest: #1a8870;
  --ifm-color-primary-light: #29d5b0;
  --ifm-color-primary-lighter: #32d8b4;
  --ifm-color-primary-lightest: #4fddbf;

  /* Code block colors */
  --ifm-code-background: #f6f8fa;
  --ifm-code-color: #24292f;

  /* Content colors */
  --ifm-background-color: #ffffff;
  --ifm-footer-background-color: #f8f9fa;
}

/* Dark mode overrides */
[data-theme='dark'] {
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: #21af90;
  --ifm-color-primary-darker: #1fa588;
  --ifm-color-primary-darkest: #1a8870;
  --ifm-color-primary-light: #29d5b0;
  --ifm-color-primary-lighter: #32d8b4;
  --ifm-color-primary-lightest: #4fddbf;

  --ifm-background-color: #0d1117;
  --ifm-footer-background-color: #161b22;
}
```

### Custom Theme Components

You can also override specific theme components by creating files in `src/theme/`:

```jsx
// src/theme/MDXComponents.js
import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import Admonition from '@theme/Admonition';

// Custom component for specifications
const SpecTable = ({ children, title }) => (
  <div className="spec-table-container">
    <h4>{title}</h4>
    <table className="spec-table">
      {children}
    </table>
  </div>
);

export default {
  ...MDXComponents,
  SpecTable,
  Admonition,
};
```

### Layout Customization

Create custom layouts by extending the default theme:

```jsx
// src/theme/Layout/index.js
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Footer from '@theme/Footer';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <Footer />
    </>
  );
}
```

## Search Configuration

### Built-in Search

Docusaurus provides two search options:

1. **Lunr.js (Local Search)**: Indexes your content locally, no external dependencies
2. **Algolia DocSearch**: Powerful hosted search service (free for open source)

### Configuring Algolia DocSearch

For technical books, Algolia provides excellent search capabilities:

```js
// docusaurus.config.js
module.exports = {
  themes: [
    [
      '@docusaurus/theme-classic',
      {
        algolia: {
          // The application ID provided by Algolia
          appId: 'YOUR_APP_ID',

          // Public API key: it is safe to commit it
          apiKey: 'YOUR_SEARCH_API_KEY',

          indexName: 'your-index-name',

          // Optional: see doc section below
          contextualSearch: true,

          // Optional: Specify domains where the navigation should occur through window.location instead on history.push. Useful when our Algolia config crawls multiple documentation sites and we want to navigate with window.location.href to them.
          externalUrlRegex: 'external\\.com|domain\\.com',

          // Optional: Replace parts of the item URLs from Algolia. Useful when using the same search index for multiple deployments using a different baseUrl. You can use regexp or string in the `from` param. For example: localhost:3000 vs myCompany.com/docs
          replaceSearchResultPathname: {
            from: '/docs/', // or as RegExp: /\/docs\//
            to: '/',
          },

          // Optional: Algolia search parameters
          searchParameters: {},

          // Optional: path for search page that enabled by default (`false` to disable it)
          searchPagePath: 'search',

          //... other Algolia params
        },
      },
    ],
  ],
};
```

### Local Search with @easyops-cn/docusaurus-search-local

For self-hosted search without external dependencies:

```js
// docusaurus.config.js
module.exports = {
  plugins: [
    [
      '@easyops-cn/docusaurus-search-local',
      {
        // Whether to index docs pages
        indexDocs: true,

        // Whether to index blog pages
        indexBlog: true,

        // Whether to index static pages
        indexPages: false,

        // Language of your documentation, see next section
        language: 'en',

        // Setting this to "none" will prevent the default CSS to be included.
        style: undefined,

        // The maximum number of search results shown to the user. Defaults to 8.
        maxSearchResults: 25,

        // lunr.js-specific settings
        lunr: {
          // When indexing your documents, their content is split into "tokens".
          // Text entered by the user will also be tokenized.
          // This setting configures the separator used to determine where to split the text into tokens.
          // By default, it splits the text at the spaces, hyphens, and dots.
          tokenizerSeparator: /[\s\-\.]+/,
        },
      },
    ],
  ],
};
```

### Search Optimization for Technical Books

For spec-driven books, optimize search with these strategies:

```js
// docusaurus.config.js
module.exports = {
  themes: [
    [
      '@docusaurus/theme-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Optimize search indexing
          editUrl: 'https://github.com/your-org/your-book/edit/main/',
          showLastUpdateAuthor: true,
          showLastUpdateTime: true,
          // Add search metadata
          remarkPlugins: [
            [require('@docusaurus/remark-plugin-npm2yarn'), {sync: true}],
          ],
        },
        algolia: {
          appId: 'YOUR_APP_ID',
          apiKey: 'YOUR_API_KEY',
          indexName: 'your-book-name',
          contextualSearch: true,
          searchParameters: {
            // Boost important content types
            facetFilters: ['type:specification OR type:requirement OR type:architecture'],
          },
        },
      },
    ],
  ],
};
```

## Essential Plugins for Technical Books

### 1. @docusaurus/plugin-content-docs

The core plugin for documentation content:

```js
// docusaurus.config.js
module.exports = {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          // Path to docs content
          path: 'docs',
          // Sidebar configuration
          sidebarPath: require.resolve('./sidebars.js'),
          // Edit URL for "Edit this page" links
          editUrl: 'https://github.com/your-org/your-book/edit/main/',
          // Show last update author
          showLastUpdateAuthor: true,
          // Show last update time
          showLastUpdateTime: true,
          // Include current version in search
          includeCurrentVersion: true,
          // Custom URL root
          routeBasePath: '/docs',
          // Versioning configuration
          versions: {
            current: {
              label: 'Next',
              path: 'next',
            },
          },
          // Additional remark plugins
          remarkPlugins: [],
          // Additional rehype plugins
          rehypePlugins: [],
          // Customize the behavior of the docs plugin
          excludeNextVersionDocs: false,
          // Only document IDs that match this pattern will be included in search
          searchMetadataTags: ['spec', 'requirement', 'architecture'],
        },
      },
    ],
  ],
};
```

### 2. @docusaurus/plugin-google-gtag

For analytics tracking:

```js
// docusaurus.config.js
module.exports = {
  plugins: [
    [
      '@docusaurus/plugin-google-gtag',
      {
        trackingID: 'GA-CODE',
        anonymizeIP: true,
      },
    ],
  ],
};
```

### 3. @docusaurus/plugin-sitemap

For SEO optimization:

```js
// docusaurus.config.js
module.exports = {
  plugins: [
    [
      '@docusaurus/plugin-sitemap',
      {
        changefreq: 'weekly',
        priority: 0.5,
        filename: 'sitemap.xml',
      },
    ],
  ],
};
```

### 4. @docusaurus/plugin-client-redirects

For URL management:

```js
// docusaurus.config.js
module.exports = {
  plugins: [
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          {
            to: '/docs/module-1/specification',
            from: ['/docs/spec', '/docs/old-spec-path'],
          },
          {
            to: '/docs/module-2/implementation',
            from: '/docs/old-implementation-path',
          },
        ],
      },
    ],
  ],
};
```

### 5. @docusaurus/plugin-pwa

For Progressive Web App features:

```js
// docusaurus.config.js
module.exports = {
  plugins: [
    [
      '@docusaurus/plugin-pwa',
      {
        debug: true,
        offlineModeActivationStrategies: [
          'appInstalled',
          'standalone',
          'queryString',
        ],
        pwaHead: [
          {
            tagName: 'link',
            rel: 'icon',
            href: '/img/favicon.ico',
          },
          {
            tagName: 'link',
            rel: 'manifest',
            href: '/manifest.json',
          },
          {
            tagName: 'meta',
            name: 'theme-color',
            content: 'rgb(37, 194, 160)',
          },
          {
            tagName: 'meta',
            name: 'apple-mobile-web-app-title',
            content: 'Docusaurus',
          },
          {
            tagName: 'meta',
            name: 'apple-mobile-web-app-status-bar-style',
            content: '#000',
          },
          {
            tagName: 'link',
            rel: 'apple-touch-icon',
            href: '/img/apple-touch-icon.png',
          },
        ],
      },
    ],
  ],
};
```

## Advanced Theming for Technical Books

### Custom CSS Classes for Documentation Types

Add custom CSS classes for different types of documentation:

```css
/* src/css/custom.css */
/* Specification blocks */
.spec-block {
  border-left: 4px solid #25c2a0;
  background-color: #f8f9fa;
  padding: 1rem;
  margin: 1rem 0;
}

.spec-requirement {
  border-left: 4px solid #ff6b6b;
  background-color: #ffe0e0;
  padding: 1rem;
  margin: 1rem 0;
}

.spec-architecture {
  border-left: 4px solid #4ecdc4;
  background-color: #e0f7fa;
  padding: 1rem;
  margin: 1rem 0;
}

/* Code block enhancements */
.code-block-spec {
  border-left: 3px solid #25c2a0;
}

/* Table styling for specifications */
.spec-table {
  width: 100%;
  border-collapse: collapse;
  margin: 1rem 0;
}

.spec-table th,
.spec-table td {
  border: 1px solid #dee2e6;
  padding: 0.75rem;
  text-align: left;
}

.spec-table th {
  background-color: #f8f9fa;
  font-weight: bold;
}

/* Breadcrumb enhancements */
.breadcrumb--spec {
  background-color: #e9ecef;
  padding: 0.5rem 1rem;
  border-radius: 0.25rem;
}
```

### Custom React Components for Technical Content

Create reusable components for common technical documentation patterns:

```jsx
// src/components/SpecRequirement.js
import React from 'react';
import Admonition from '@theme/Admonition';

export default function SpecRequirement({ id, type = 'functional', children }) {
  const title = `${type.charAt(0).toUpperCase() + type.slice(1)} Requirement ${id}`;
  const typeColor = type === 'functional' ? 'info' : 'caution';

  return (
    <Admonition type={typeColor} title={title}>
      {children}
    </Admonition>
  );
}
```

```jsx
// src/components/ArchitectureDiagram.js
import React from 'react';

export default function ArchitectureDiagram({ children, title }) {
  return (
    <div className="architecture-diagram">
      <h4>{title}</h4>
      <div className="diagram-container">
        {children}
      </div>
    </div>
  );
}
```

### Syntax Highlighting Customization

Customize syntax highlighting for specific use cases:

```js
// docusaurus.config.js
module.exports = {
  themes: [
    [
      '@docusaurus/theme-classic',
      {
        prism: {
          theme: require('prism-react-renderer/themes/github'),
          darkTheme: require('prism-react-renderer/themes/dracula'),
          // Additional languages
          additionalLanguages: [
            'java',
            'scala',
            'csharp',
            'python',
            'json',
            'yaml',
            'bash',
            'docker',
            'sql',
            'graphql',
            'rust',
            'toml',
            'hcl',
            'protobuf',
            'wasm',
            'regex',
            'diff',
            'ts',
            'tsx',
            'jsx',
            'js',
            'html',
            'css',
            'docker',
            'git',
            'go',
            'java',
            'kotlin',
            'objectivec',
            'swift',
            'php',
            'perl',
            'python',
            'r',
            'ruby',
            'rust',
            'scala',
            'sql',
            'c',
            'cpp',
            'csharp',
            'fsharp',
            'haskell',
            'erlang',
            'elixir',
            'clojure',
            'scheme',
            'lisp',
            'matlab',
            'octave',
            'julia',
            'dart',
            'assembly',
            'llvm',
            'webassembly',
            'llvm-ir',
            'wasm',
            'regex',
            'diff',
            'ignore',
            'javadoc',
            'javaex',
            'jsdoc',
            'jolie',
            'julia-repl',
            'kdb',
            'kernel',
            'kumir',
            'kusto',
            'latex',
            'ldif',
            'leaf',
            'lean',
            'less',
            'lisp',
            'livecodeserver',
            'livescript',
            'llvm',
            'lsl',
            'lua',
            'makefile',
            'markdown',
            'mathematica',
            'matlab',
            'maxima',
            'mel',
            'mercury',
            'mipsasm',
            'mizar',
            'perl',
            'mojolicious',
            'monkey',
            'moonscript',
            'n1ql',
            'nestedtext',
            'nginx',
            'nim',
            'nix',
            'node-repl',
            'nsis',
            'objectivec',
            'ocaml',
            'openscad',
            'oxygene',
            'parser3',
            'pf',
            'pgsql',
            'php',
            'php-template',
            'plsql',
            'powershell',
            'processing',
            'profile',
            'prolog',
            'properties',
            'protobuf',
            'puppet',
            'purebasic',
            'python-repl',
            'q',
            'qml',
            'r',
            'reasonml',
            'rib',
            'roboconf',
            'routeros',
            'rsl',
            'ruby',
            'ruleslanguage',
            'rust',
            'sas',
            'scala',
            'scheme',
            'scilab',
            'scss',
            'shell',
            'smali',
            'smalltalk',
            'sml',
            'sqf',
            'sql',
            'stan',
            'stata',
            'step21',
            'stylus',
            'subunit',
            'swift',
            'taggerscript',
            'yaml',
            'tap',
            'tcl',
            'thrift',
            'tp',
            'twig',
            'typescript',
            'vala',
            'vbnet',
            'vbscript',
            'vbscript-html',
            'verilog',
            'vhdl',
            'vim',
            'x86asm',
            'xl',
            'xquery',
            'zephir'
          ],
          // Default language
          defaultLanguage: 'markdown',
          // Show line numbers by default
          showLineNumbers: false,
        },
      },
    ],
  ],
};
```

## Performance Optimization

### Image Optimization

For technical books with diagrams and images:

```jsx
// Use Docusaurus' built-in image optimization
import Image from '@theme/IdealImage';

export default function ArchitecturePage() {
  return (
    <div>
      <h1>System Architecture</h1>
      <Image
        img={require('./img/architecture-diagram.png')}
        alt="System Architecture Diagram"
        className="architecture-diagram-img"
      />
    </div>
  );
}
```

### Code Splitting and Lazy Loading

Configure webpack for optimal loading:

```js
// docusaurus.config.js
module.exports = {
  themes: [
    [
      '@docusaurus/theme-classic',
      {
        // Theme options...
      },
    ],
  ],
  // Custom webpack configuration
  webpack: {
    jsLoader: (isServer) => ({
      loader: require.resolve('swc-loader'),
      options: {
        jsc: {
          parser: {
            syntax: 'typescript',
            tsx: true,
          },
          target: 'es2017',
        },
        module: {
          type: isServer ? 'commonjs' : 'es6',
        },
      },
    }),
  },
};
```

## Accessibility Features

Ensure your technical book is accessible:

```js
// docusaurus.config.js
module.exports = {
  themes: [
    [
      '@docusaurus/theme-classic',
      {
        // Theme options...
      },
    ],
  ],
  // Accessibility options
  themeConfig: {
    // Color mode with respect for user preference
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    // Additional accessibility options
    metadata: [
      {
        name: 'keywords',
        content: 'documentation, technical writing, docusaurus, spec-driven, technical book'
      },
      {
        name: 'description',
        content: 'Comprehensive guide to creating spec-driven technical books with Docusaurus'
      }
    ],
  },
};
```

## Summary

Theming, search, and plugins are essential for creating professional technical books with Docusaurus. Custom theming allows you to match your brand and content style, while proper search configuration ensures readers can find information efficiently. The right combination of plugins enhances functionality and user experience. The next chapter will cover building and deploying your Docusaurus site to GitHub Pages.