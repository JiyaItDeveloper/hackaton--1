---
sidebar_position: 3
---

# Chapter 2: Project Structure & Routing

## Learning Goals
- Define folder structure for modules and chapters in Docusaurus
- Configure sidebar navigation for spec-driven books
- Understand versioning and routing in Docusaurus

## Recommended Project Structure for Spec-Driven Books

For spec-driven documentation, a well-organized project structure is crucial for maintainability and navigation. Here's a recommended structure:

```
my-spec-driven-book/
├── blog/                           # Optional blog posts about the project
├── docs/                           # All documentation content
│   ├── introduction/               # Introductory content
│   │   ├── getting-started.md
│   │   └── overview.md
│   ├── module-1-specification/     # Module 1 specification
│   │   ├── index.md               # Module overview
│   │   ├── requirements.md
│   │   ├── architecture.md
│   │   └── acceptance-criteria.md
│   ├── module-1-plan/              # Module 1 implementation plan
│   │   ├── approach.md
│   │   ├── technology-stack.md
│   │   ├── implementation-steps.md
│   │   └── risks.md
│   ├── module-1-tasks/             # Module 1 tasks
│   │   ├── task-list.md
│   │   ├── implementation-notes.md
│   │   └── test-cases.md
│   ├── module-2-specification/     # Module 2 specification
│   │   ├── index.md
│   │   └── ...
│   └── module-2-plan/
│       └── ...
├── src/                            # Custom React components and pages
│   ├── components/                 # Reusable components
│   │   ├── ArchitectureDiagram/
│   │   ├── CodeComparison/
│   │   └── SpecTable/
│   ├── pages/                      # Custom pages
│   └── css/                        # Custom styles
├── static/                         # Static assets
│   ├── img/                        # Images and diagrams
│   ├── specs/                      # Raw specification files
│   └── assets/                     # Other assets
├── docusaurus.config.js            # Main site configuration
├── sidebars.js                     # Navigation structure
├── package.json                    # Dependencies
└── README.md                       # Project documentation
```

## Organizing Modules and Chapters

### Hierarchical Organization

For spec-driven books, organize content in a clear hierarchy:

1. **Top-level**: Major modules or components
2. **Second-level**: Specification, Plan, Tasks for each module
3. **Third-level**: Detailed content within each category

### Example Structure for a Technical Book

```
docs/
├── introduction/
│   ├── index.md
│   ├── philosophy.md
│   └── how-to-use-this-book.md
├── module-1-core-concepts/
│   ├── index.md
│   ├── specification/
│   │   ├── requirements.md
│   │   ├── architecture.md
│   │   └── constraints.md
│   ├── plan/
│   │   ├── approach.md
│   │   ├── timeline.md
│   │   └── resources.md
│   └── implementation/
│       ├── setup.md
│       ├── examples.md
│       └── best-practices.md
├── module-2-advanced-topics/
│   ├── index.md
│   └── ...
└── appendix/
    ├── glossary.md
    ├── references.md
    └── faq.md
```

## Sidebar Configuration

The `sidebars.js` file controls the navigation structure. For spec-driven books, you want a clear, organized sidebar:

```js
// sidebars.js
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['introduction/index', 'introduction/how-to-use'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: Core Concepts',
      items: [
        'module-1-core-concepts/index',
        {
          type: 'category',
          label: 'Specification',
          items: [
            'module-1-core-concepts/specification/requirements',
            'module-1-core-concepts/specification/architecture',
            'module-1-core-concepts/specification/constraints'
          ],
        },
        {
          type: 'category',
          label: 'Implementation Plan',
          items: [
            'module-1-core-concepts/plan/approach',
            'module-1-core-concepts/plan/timeline',
            'module-1-core-concepts/plan/resources'
          ],
        },
        {
          type: 'category',
          label: 'Implementation',
          items: [
            'module-1-core-concepts/implementation/setup',
            'module-1-core-concepts/implementation/examples',
            'module-1-core-concepts/implementation/best-practices'
          ],
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Advanced Topics',
      items: ['module-2-advanced-topics/index', '...'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/glossary',
        'appendix/references',
        'appendix/faq'
      ],
      collapsed: true,
    },
  ],
};
```

## Frontmatter Configuration for Navigation

Use frontmatter to control how pages appear in navigation:

```markdown
---
# For main module pages
sidebar_label: 'Module 1: Core Concepts'
sidebar_position: 1
title: 'Module 1 Specification'
description: 'Complete specification for the Core Concepts module'
keywords: [specification, requirements, architecture]
---

# Module 1: Core Concepts

Content here...
```

```markdown
---
# For specification documents
sidebar_label: 'Requirements'
sidebar_position: 2
title: 'Module 1 Requirements'
description: 'Detailed requirements for the Core Concepts module'
keywords: [requirements, functional, non-functional]
---

# Requirements

Content here...
```

## Versioning Configuration

For spec-driven books that evolve over time, configure versioning in `docusaurus.config.js`:

```js
// docusaurus.config.js
module.exports = {
  title: 'My Spec-Driven Book',
  tagline: 'Documentation for spec-driven development',
  url: 'https://your-book.com',
  baseUrl: '/',
  organizationName: 'your-org',
  projectName: 'your-book',

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-org/your-book/edit/main/',
          // Versioning configuration
          versions: {
            current: {
              label: 'Next',
              path: 'next',
              banner: 'unreleased',
            },
            '1.0.0': {
              label: '1.0.0',
              path: '1.0.0',
            },
            '0.9.0': {
              label: '0.9.0',
              path: '0.9.0',
              banner: 'unmaintained',
            },
          },
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://github.com/your-org/your-book/edit/main/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};
```

## Routing and URL Structure

Docusaurus automatically generates URLs based on file structure:

- `docs/introduction/index.md` → `/docs/introduction/`
- `docs/module-1-core-concepts/specification/requirements.md` → `/docs/module-1-core-concepts/specification/requirements`
- `docs/module-1-core-concepts/index.md` → `/docs/module-1-core-concepts/`

You can customize this with the `slug` frontmatter property:

```markdown
---
slug: /modules/core-concepts/spec/requirements
---

# Requirements
```

## Custom Navigation Components

For complex spec-driven books, you might want to create custom navigation components:

```jsx
// src/components/SpecStatus.js
import React from 'react';

export default function SpecStatus({ status, lastUpdated }) {
  const statusColors = {
    draft: 'gray',
    review: 'orange',
    approved: 'green',
    deprecated: 'red',
  };

  return (
    <div style={{
      display: 'inline-block',
      padding: '4px 8px',
      borderRadius: '4px',
      backgroundColor: `var(--ifm-color-${statusColors[status] || 'gray'})`,
      color: 'white',
      fontSize: '0.8rem',
      textTransform: 'uppercase',
      fontWeight: 'bold',
      marginRight: '8px',
    }}>
      {status} {lastUpdated && `(${lastUpdated})`}
    </div>
  );
}
```

Then use it in your documentation:

```mdx
---
sidebar_label: 'Requirements'
sidebar_position: 2
title: 'Module 1 Requirements'
---

import SpecStatus from '@site/src/components/SpecStatus';

# Requirements

<SpecStatus status="approved" lastUpdated="2023-12-01" />

This document outlines the approved requirements for Module 1...
```

## Best Practices for Spec-Driven Book Structure

1. **Consistent Naming**: Use consistent naming conventions for all files and folders
2. **Clear Hierarchy**: Maintain a clear hierarchy that reflects the logical organization of your specifications
3. **Cross-References**: Use Docusaurus' linking capabilities to reference related specifications
4. **Metadata Consistency**: Apply consistent frontmatter across similar document types
5. **Version Awareness**: Plan for versioning from the start if your specifications will evolve
6. **Search Optimization**: Use descriptive titles and keywords to improve searchability

## Summary

Proper project structure and routing are essential for spec-driven books. The hierarchical organization with clear navigation enables readers to easily find and understand specifications, plans, and implementation details. The next chapter will cover how to write effective modules and chapters using Markdown.