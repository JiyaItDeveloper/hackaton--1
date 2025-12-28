---
sidebar_position: 4
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Chapter 3: Writing Modules & Chapters in Markdown

## Learning Goals
- Demonstrate how to author module-wise chapters using pure Markdown
- Learn about frontmatter, links, and embeds in Docusaurus
- Understand best practices for technical writing in Markdown

## Markdown Syntax for Technical Documentation

### Basic Markdown Elements

Docusaurus supports standard Markdown syntax with additional features for technical documentation:

```markdown
# Module Title (H1 - Use only once per document)

## Section Title (H2 - Major sections)

### Subsection Title (H3 - Subsections)

#### Minor Heading (H4 - Detailed subsections)

This is a paragraph with **bold text**, *italic text*, and `inline code`.

> This is a blockquote or note section.
> It can span multiple lines.

- Unordered list item 1
- Unordered list item 2
  - Nested item

1. Ordered list item 1
2. Ordered list item 2

`inline code` and ```code blocks``` are essential for technical documentation.
```

### Code Blocks with Syntax Highlighting

Code blocks in Docusaurus support syntax highlighting and additional features:

````markdown
```js
// JavaScript example
function example() {
  const x = 1;
  return x * 2;
}
```

```python
# Python example
def example():
    x = 1
    return x * 2
```

```bash title="Installation Command"
npm install docusaurus
```

```js title="Example.js" {1,3-5} showLineNumbers
// This code block has:
// - A title
// - Line numbers
// - Highlighted lines 1 and 3-5
function example() {
  const x = 1; // This line is highlighted
  console.log('Processing...'); // This line is highlighted
  const y = x * 2; // This line is highlighted
  return y; // This line is highlighted
}
```
````

### Tables for Specifications

Tables are useful for documenting requirements, specifications, and comparisons:

```markdown
| Requirement ID | Description | Priority | Status |
|---|---|---|---|
| REQ-001 | User authentication | High | Implemented |
| REQ-002 | Data encryption | Critical | In Progress |
| REQ-003 | API rate limiting | Medium | Planned |

| Feature | Supported | Notes |
|---|---|---|
| **Authentication** | ✅ | OAuth 2.0, JWT tokens |
| **Authorization** | ✅ | Role-based access control |
| **Encryption** | ✅ | AES-256 |
```

### Admonitions for Important Information

Docusaurus supports special blocks for important information:

```markdown
:::note
This is a note about important information.
:::

:::tip
This is a helpful tip for the reader.
:::

:::info
This provides additional information.
:::

:::caution
This warns about potential issues.
:::

:::danger
This alerts about critical problems.
:::
```

## Frontmatter for Documentation Metadata

Frontmatter is crucial for Docusaurus documentation and provides metadata for your documents:

```markdown
---
# Basic metadata
title: 'Module 1: Core Concepts Specification'
description: 'Complete specification document for the Core Concepts module'
keywords: [specification, requirements, architecture, module-1]

# Navigation configuration
sidebar_label: 'Core Concepts Spec'
sidebar_position: 1
sidebar_class_name: 'core-concepts-spec'

# Versioning
last_update:
  date: 2023-12-15
  author: 'John Doe'
  version: '1.2.0'

# Custom properties
custom_edit_url: 'https://github.com/your-org/docs/edit/main/docs/module-1/spec.md'
tags: [specification, requirements, architecture]
---

# Module 1: Core Concepts Specification

Content goes here...
```

### Advanced Frontmatter Options

```markdown
---
# SEO and social sharing
title: 'Module 1: Core Concepts Specification'
description: 'Complete specification document outlining requirements and architecture for the Core Concepts module'
image: '/img/module-1-social-card.png'

# Navigation
sidebar_label: 'Specification'
sidebar_position: 1
sidebar_class_name: 'module-1-spec'
hide_table_of_contents: false

# Content control
draft: false
pagination_next: null
pagination_prev: null

# Custom behavior
custom_edit_url: 'https://github.com/your-org/docs/edit/main/docs/module-1/spec.md'
---
# Module 1: Core Concepts Specification

Content here...
```

## Internal and External Linking

### Linking to Other Documentation Pages

```markdown
// Link to another document in the same project
See the [Implementation Guide](../module-1-implementation/guide.md) for setup instructions.

// Link to a document with a custom anchor
For more details, see [Security Considerations](./security.md#authentication).

// Link with a title attribute
[API Reference](./api-reference.md "Complete API documentation")

// Link to a document in a different section
Check out the [Testing Strategy](../module-2-testing/strategy.md) in Module 2.
```

### Linking to Specific Sections

```markdown
# Module Architecture

## Component Overview

Content about components...

## Data Flow

Content about data flow...

## Security Considerations

Content about security...

// Later in the document, you can link to specific sections:
See [Component Overview](#component-overview) for details about system components.
For data handling, refer to [Data Flow](#data-flow).
Security requirements are outlined in [Security Considerations](#security-considerations).
```

## Embedding Images and Assets

### Adding Images

```markdown
// Basic image
![Diagram of system architecture](/img/architecture-diagram.png)

// Image with alt text and title
![System Architecture](/img/architecture-diagram.png "System Architecture Diagram")

// Responsive image with custom class
![Flow Chart](/img/flow-chart.svg "Process Flow Chart")

// Image in a specific location
![Component Diagram](./diagrams/component-diagram.png "Component Interaction Diagram")
```

### Image Optimization

```text
// Docusaurus supports image directives with additional options
import Image from '@theme/IdealImage';

<Image
  img={require('./path/to/image.png')}
  alt="Alternative text"
  className="custom-class"
  style={{border: '1px solid #ccc'}}
/>
```

## Creating Technical Diagrams and Tables

### Complex Tables for Specifications

```markdown
| Component | Type | Interface | Dependencies | Status |
|---|---|---|---|---|
| **User Service** | Microservice | REST API | Auth Service | ✅ Active |
| **Auth Service** | Microservice | gRPC | User DB | ✅ Active |
| **User DB** | Database | PostgreSQL | None | ✅ Active |
| **Cache Layer** | Service | Redis | All Services | ⏳ Planned |

### API Specification Table

| Endpoint | Method | Description | Auth Required |
|---|---|---|---|
| `/api/users` | `GET` | Get all users | ✅ Bearer Token |
| `/api/users/:id` | `GET` | Get user by ID | ✅ Bearer Token |
| `/api/users` | `POST` | Create new user | ❌ None |
```

### Mathematical Expressions

For mathematical content, use LaTeX syntax within Markdown:

```markdown
The system response time is calculated as:

$$
T_{total} = T_{network} + T_{processing} + T_{database}
$$

Where:
- $T_{network}$ is the network latency
- $T_{processing}$ is the processing time
- $T_{database}$ is the database query time
```

## Cross-Module References

### Referencing Other Modules

```markdown
// Reference to another module's specification
For authentication requirements, see the [Authentication Module Specification](../module-auth/spec.md).

// Reference with context
The Core Concepts module (see [Module 1 Spec](../module-1/spec.md)) defines the foundation for all other modules.

// Cross-reference with anchor
For security implementation details, refer to the [Security Module](../module-security/implementation.md#encryption).
```

## Writing Effective Technical Content

### Document Structure Best Practices

```markdown
---
title: 'Module X: Feature Specification'
description: 'Detailed specification for Feature Y in Module X'
keywords: [specification, feature, requirements, module-x]
---

# Module X: Feature Specification

## Overview

Brief introduction to the feature and its purpose.

## Requirements

### Functional Requirements

1. **REQ-XXX**: The system SHALL...
2. **REQ-XXX**: The system MUST...
3. **REQ-XXX**: The system SHOULD...

### Non-Functional Requirements

- Performance: The system must respond within...
- Security: All data must be encrypted...
- Scalability: Support up to X concurrent users...

## Architecture

### Component Diagram

![Architecture Diagram](/img/feature-architecture.png)

### Data Flow

1. User initiates request
2. Request validated by...
3. Data processed by...
4. Response returned to user

## Implementation Details

### API Endpoints

| Endpoint | Method | Description |
|---|---|---|
| `/api/feature` | `GET` | Retrieve feature data |

### Configuration

Required configuration parameters:

```yaml
feature:
  enabled: true
  timeout: 30000
  max_connections: 100
```

## Testing Strategy

### Unit Tests

- Test individual components in isolation
- Mock external dependencies

### Integration Tests

- Test component interactions
- Verify data flow

## Security Considerations

- Authentication requirements
- Authorization rules
- Data encryption

## Performance Guidelines

- Response time requirements
- Resource usage limits

## Deployment

### Prerequisites

- Node.js 16+
- PostgreSQL 12+
- Redis 6+

### Configuration

Steps to configure the feature in different environments.
```

## Using Docusaurus-Specific Features

### Collapsible Sections

:::note
Example of collapsible configuration:

```yaml
# Detailed configuration options
feature:
  advanced:
    timeout: 30000
    retry_count: 3
    circuit_breaker:
      enabled: true
      threshold: 5
```

:::

Additional details about configuration...

### Tabs for Multiple Languages/Approaches

<Tabs>
  <TabItem value="javascript" label="JavaScript">
    ```js
    const result = await api.getData();
    console.log(result);
    ```
  </TabItem>
  <TabItem value="python" label="Python">
    ```py
    result = api.get_data()
    print(result)
    ```
  </TabItem>
  <TabItem value="curl" label="cURL">
    ```bash
    curl -X GET https://api.example.com/data
    ```
  </TabItem>
</Tabs>

This shows how to implement Tabs in MDX.

## Best Practices for Spec-Driven Documentation

### 1. Consistent Formatting

```markdown
// Good: Consistent heading hierarchy
# Module Title
## Requirements
### Functional Requirements
#### Specific Requirement
## Architecture
### Components
```

### 2. Clear Requirement Numbering

```markdown
### Functional Requirements

- **REQ-001**: The system SHALL authenticate users
- **REQ-002**: The system MUST encrypt sensitive data
- **REQ-003**: The system SHOULD log all access attempts

### Non-Functional Requirements

- **NFR-001**: Response time MUST be under 2 seconds
- **NFR-002**: System MUST be available 99.9% of the time
```

### 3. Versioning Information

```markdown
---
title: 'Module X Specification'
description: 'Version 2.1 of Module X specification'
version: '2.1.0'
last_updated: '2023-12-15'
status: 'approved'
---

# Module X Specification (v2.1.0)

:::info
This specification was last updated on December 15, 2023.
Previous version: [v2.0.0](./spec-v2.0.0.md)
:::
```

## Summary

Writing effective modules and chapters in Markdown requires understanding both basic Markdown syntax and Docusaurus-specific features. Proper use of frontmatter, consistent formatting, and appropriate linking creates well-structured, maintainable documentation. The next chapter will cover theming, search, and plugins for enhancing your technical book.