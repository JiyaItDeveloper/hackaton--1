---
sidebar_position: 2
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 1: Docusaurus Fundamentals

## Learning Goals
- Understand Docusaurus architecture and core concepts
- Learn about MD/MDX workflow for technical documentation
- Identify why Docusaurus is suitable for spec-driven books

## Introduction to Docusaurus

Docusaurus is a modern static site generator designed specifically for building documentation websites. It's built with performance, scalability, and developer experience in mind, making it an ideal choice for technical authoring.

### Core Architecture

Docusaurus follows a plugin-based architecture that allows for extensive customization while maintaining simplicity for common use cases:

1. **Static Site Generation**: Docusaurus pre-builds your entire site as static HTML, CSS, and JavaScript files, ensuring fast loading times and excellent SEO.

2. **Plugin System**: Core functionality is provided through plugins:
   - `@docusaurus/plugin-content-docs`: Handles documentation pages
   - `@docusaurus/plugin-content-blog`: Manages blog posts
   - `@docusaurus/plugin-content-pages`: Supports custom pages
   - `@docusaurus/plugin-client-redirects`: Handles redirects
   - `@docusaurus/plugin-sitemap`: Generates sitemap.xml

3. **React-based Components**: Built on React, allowing for rich, interactive components within documentation.

### MD vs MDX Workflow

Docusaurus supports both standard Markdown (MD) and MDX (Markdown + JSX):

#### Standard Markdown (MD)
```markdown
# My Document

This is a simple paragraph with **bold** and *italic* text.

- List item 1
- List item 2
- List item 3
```

#### MDX (Markdown + JSX)
```mdx
# My Document

This is a paragraph with <strong>bold</strong> and <em>italic</em> text.

## Code Block

```js
function example() {
  return 'Hello, world!';
}
```
```

### Why Docusaurus for Spec-Driven Books

#### 1. **Structured Documentation**
Docusaurus naturally supports organizing content in a hierarchical structure, which aligns perfectly with the modular nature of spec-driven development. Each specification, plan, and task can be organized in its own section with clear navigation.

#### 2. **Versioning Support**
For spec-driven books that evolve over time, Docusaurus provides built-in versioning capabilities, allowing you to maintain multiple versions of your specifications and plans.

```js
// Example docusaurus.config.js with versioning
module.exports = {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/facebook/docusaurus/edit/master/website/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
          // Enable versioning
          versions: {
            current: {
              label: 'Next',
              path: 'next',
            },
          },
        },
      },
    ],
  ],
};
```

#### 3. **Frontmatter Support**
Docusaurus uses YAML frontmatter to add metadata to your documents, which is perfect for spec-driven documentation:

```markdown
---
title: Module 1 Specification
sidebar_label: Specification
sidebar_position: 1
description: Detailed specification for Module 1
keywords: [specification, requirements, module]
---

# Module 1 Specification

Content goes here...
```

#### 4. **Powerful Search**
Docusaurus integrates with Algolia for excellent search functionality, making it easy to find specific requirements, specifications, or implementation details across your spec-driven book.

#### 5. **Code Integration**
For technical books that include code examples (which spec-driven development often does), Docusaurus provides excellent syntax highlighting and code block features:

```js
// Example code block with line highlighting
function example() {
  const x = 1; // highlight-line
  const y = 2;
  return x + y;
}
```

### Docusaurus Project Structure

A typical Docusaurus project for spec-driven books follows this structure:

```
my-book/
├── blog/           # Optional blog posts
├── docs/           # Documentation files (specs, plans, tasks)
│   ├── module-1/
│   │   ├── spec.md
│   │   ├── plan.md
│   │   └── tasks.md
│   └── module-2/
│       ├── spec.md
│       ├── plan.md
│       └── tasks.md
├── src/
│   ├── components/ # Custom React components
│   ├── pages/      # Custom pages
│   └── css/        # Custom CSS
├── static/         # Static assets (images, files)
├── docusaurus.config.js  # Site configuration
├── package.json
└── sidebars.js     # Navigation structure
```

### MDX Capabilities for Technical Books

MDX allows you to embed React components directly in your Markdown, which is incredibly powerful for technical documentation:

````md
```mdx
# API Specification

<Tabs>
  <TabItem value="javascript" label="JavaScript">
    ```js
    const result = await api.call();
    ```
  </TabItem>
  <TabItem value="python" label="Python">
    ```py
    result = api.call()
    ```
  </TabItem>
</Tabs>

This enables you to create interactive examples, API references, and complex documentation structures that are difficult to achieve with standard Markdown.

<Tabs>
  <TabItem value="javascript" label="JavaScript">
    ```js
    const result = await api.call();
    ```
  </TabItem>
  <TabItem value="python" label="Python">
    ```py
    result = api.call()
    ```
  </TabItem>
</Tabs>

## Summary

Docusaurus provides an excellent foundation for spec-driven technical books with its structured approach, powerful search, versioning capabilities, and support for both MD and MDX formats. The next chapter will explore how to organize your project structure and routing for maximum effectiveness.