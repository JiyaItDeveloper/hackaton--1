---
sidebar_position: 6
---

# Chapter 5: Build & Deployment to GitHub Pages

## Learning Goals
- Learn the complete process to build Docusaurus sites
- Understand testing procedures before deployment
- Master deployment to GitHub Pages for public access

## Building Your Docusaurus Site

### Local Development

Before building for production, test your site locally:

```bash
# Install dependencies (run once)
npm install

# Start local development server
npm start

# This starts a local server at http://localhost:3000
# with hot reloading for real-time editing
```

The development server automatically reloads when you make changes to:
- Markdown files in the `docs/` directory
- Configuration files (`docusaurus.config.js`, `sidebars.js`)
- React components in `src/`
- CSS files in `src/css/`

### Production Build

To create an optimized production build:

```bash
# Build the site for production
npm run build

# This creates a static site in the "build" directory
# The site is optimized for performance and SEO
```

The build process includes:
- Bundling and minifying JavaScript and CSS
- Optimizing images
- Generating static HTML files for all pages
- Creating a service worker for offline functionality (if PWA plugin is enabled)
- Generating a sitemap.xml file
- Preloading critical resources

### Build Configuration Options

You can customize the build process in `docusaurus.config.js`:

```js
// docusaurus.config.js
module.exports = {
  // ... other config
  trailingSlash: true, // Add / to the end of URLs
  onBrokenLinks: 'throw', // Handle broken links
  onBrokenMarkdownLinks: 'warn', // Handle broken markdown links
  favicon: 'img/favicon.ico',

  // Custom build options
  customFields: {
    buildTimestamp: new Date().toISOString(),
  },

  // Sitemap configuration
  sitemap: {
    changefreq: 'weekly',
    priority: 0.5,
    filename: 'sitemap.xml',
  },
};
```

### Testing the Build Locally

Before deploying, test the production build locally:

```bash
# Build the site
npm run build

# Serve the built site locally
npm run serve

# This serves the site at http://localhost:3000
# with the same configuration as production
```

## Pre-Deployment Testing

### 1. Content Verification

Check all content before deployment:

```bash
# Verify all internal links work
npx linkinator http://localhost:3000 --recurse

# Check for broken links in markdown files
npx markdown-link-check "**/*.md"
```

### 2. Build Validation

Ensure the build completes without errors:

```bash
# Build with verbose output to catch warnings
npm run build -- --verbose

# Check for common issues
npm run build && echo "Build successful" || echo "Build failed"
```

### 3. Cross-Browser Testing

Test your site across different browsers:

```bash
# Use browserstack or similar for cross-browser testing
# Or use automated tools like:
npx pa11y http://localhost:3000 --include-warnings
```

### 4. Performance Testing

Check site performance:

```bash
# Use Lighthouse for performance audit
npx lighthouse http://localhost:3000 --view

# Or use WebPageTest
# https://www.webpagetest.org/
```

### 5. Accessibility Testing

Ensure your site is accessible:

```bash
# Use axe-core for accessibility testing
npx axe-cli http://localhost:3000 --include '#content'
```

## GitHub Pages Deployment Setup

### 1. Repository Configuration

First, ensure your repository is properly set up:

```bash
# Initialize git repository (if not already done)
git init

# Add all files
git add .

# Initial commit
git commit -m "Initial commit: Docusaurus documentation site"

# Connect to your GitHub repository
git remote add origin https://github.com/your-username/your-repo-name.git

# Push to GitHub
git push -u origin main
```

### 2. GitHub Pages Configuration

Configure GitHub Pages in your repository settings:

1. Go to your repository on GitHub
2. Click on "Settings" tab
3. In the left sidebar, click "Pages"
4. Under "Build and deployment", set:
   - Source: Deploy from a branch
   - Branch: `gh-pages`, `/ (root)` or `/docs` depending on your setup

### 3. Deployment Configuration

Update your `docusaurus.config.js` with GitHub Pages settings:

```js
// docusaurus.config.js
const config = {
  // Set the production URL of your site
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/your-repo-name/', // Replace with your repository name

  // GitHub pages deployment config
  organizationName: 'your-username', // Usually your GitHub username
  projectName: 'your-repo-name', // Usually your repository name
  deploymentBranch: 'gh-pages', // Branch to deploy to
  trailingSlash: false, // Set based on your preference

  // ... rest of your config
};
```

## Deployment Methods

### Method 1: Manual Deployment with GitHub Actions (Recommended)

Create a GitHub Actions workflow for automated deployment:

```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]  # Trigger deployment on pushes to main branch
  pull_request:
    branches: [main]  # Optional: Run tests on pull requests

jobs:
  test-deploy:
    if: github.event_name == 'pull_request'  # Only run for pull requests
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Test build website
        run: npm run build

  deploy:
    if: github.event_name == 'push'  # Only deploy on pushes to main
    runs-on: ubuntu-latest
    concurrency:
      group: ${{ github.workflow }}-${{ github.ref }}
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Build website
        run: npm run build

      # Popular action to deploy to GitHub Pages:
      # Docs: https://github.com/peaceiris/actions-gh-pages#%EF%B8%8F-docusaurus
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

### Method 2: CLI Deployment

Use the Docusaurus CLI for manual deployment:

```bash
# Install the deployment CLI tool
npm install --save-dev gh-pages

# Add deploy script to package.json
# Add this to your scripts section:
# "scripts": {
#   "deploy": "docusaurus deploy"
# }

# Deploy to GitHub Pages
GIT_USER=your-github-username USE_SSH=false npm run deploy

# Or set environment variables
export GIT_USER=your-github-username
export DEPLOYMENT_BRANCH=gh-pages
npm run deploy
```

### Method 3: Git Branch Deployment

Deploy directly to the `gh-pages` branch:

```bash
# Build the site
npm run build

# Navigate to the build directory
cd build

# Initialize a new git repository
git init

# Add remote origin
git remote add origin https://github.com/your-username/your-repo-name.git

# Add all files
git add .

# Commit changes
git commit -m "Deploy to GitHub Pages"

# Push to gh-pages branch
git push -f origin main:gh-pages

# Return to main directory
cd ..
```

## GitHub Actions Workflow Configuration

### Complete GitHub Actions Workflow

Create a comprehensive workflow that handles building, testing, and deploying:

```yaml
# .github/workflows/ci-cd.yml
name: CI/CD Pipeline

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

# Sets permissions of the GITHUB_TOKEN to allow deployment to GitHub Pages
permissions:
  contents: read
  pages: write
  id-token: write

# Allow only one concurrent deployment, skipping runs queued between the run in-progress and latest queued.
# Cancel in-progress runs when a new workflow is triggered
concurrency:
  group: "pages"
  cancel-in-progress: true

jobs:
  # Build job
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4
      - name: Setup Node
        uses: actions/setup-node@v4
        with:
          node-version: "18"
          cache: npm
      - name: Setup Pages
        uses: actions/configure-pages@v4
      - name: Install dependencies
        run: npm ci
      - name: Build with Docusaurus
        run: |
          npm run build
      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: build

  # Deployment job
  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

### Environment-Specific Deployment

For different environments (staging, production):

```yaml
# .github/workflows/deploy.yml
name: Deploy Documentation

on:
  push:
    branches: [main, staging]
  pull_request:
    branches: [main]

jobs:
  deploy-staging:
    if: github.ref == 'refs/heads/staging'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm
      - run: npm ci
      - run: npm run build
        env:
          DEPLOYMENT_ENV: staging
      - name: Deploy to staging
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
          destination_dir: staging

  deploy-production:
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm
      - run: npm ci
      - run: npm run build
        env:
          DEPLOYMENT_ENV: production
      - name: Deploy to production
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

## Custom Domain Configuration

If you want to use a custom domain:

### 1. Configure CNAME in GitHub

```bash
# Create CNAME file in static directory
echo "yourdomain.com" > static/CNAME
```

### 2. Update DNS Settings

Add DNS records to point to GitHub Pages:

```
Type: A
Name: @
Value: 185.199.108.153

Type: A
Name: @
Value: 185.199.109.153

Type: A
Name: @
Value: 185.199.110.153

Type: A
Name: @
Value: 185.199.111.153

Type: CNAME
Name: www
Value: your-username.github.io
```

## Post-Deployment Verification

### 1. Site Availability Check

Verify your site is accessible:

```bash
# Check if site is accessible
curl -I https://your-username.github.io/your-repo-name/

# Or use a tool like httpie
http https://your-username.github.io/your-repo-name/
```

### 2. Search Functionality Test

Test that search is working properly:

```bash
# For Algolia search, verify the API key works
# For local search, ensure the search index is built
```

### 3. Link Verification

Check all links after deployment:

```bash
# Use linkinator to check deployed site
npx linkinator https://your-username.github.io/your-repo-name/ --recurse
```

## Performance Optimization Post-Deployment

### 1. Image Optimization

Optimize images for faster loading:

```bash
# Use tools like ImageOptim, TinyPNG, or command line tools
# Install imagemin for automated optimization
npm install --save-dev imagemin imagemin-pngquant imagemin-mozjpeg
```

### 2. Bundle Analysis

Analyze bundle size:

```bash
# Install bundle analyzer
npm install --save-dev @docusaurus/plugin-client-redirects

# Add to docusaurus.config.js
plugins: [
  [
    '@docusaurus/plugin-client-redirects',
    {
      // ... redirect options
    },
  ],
  // Bundle analyzer for development
  process.env.NODE_ENV === 'development' && [
    '@docusaurus/plugin-webpack-bundle-analyzer',
    {
      analyzerMode: 'static',
      reportFilename: 'bundle-report.html',
    },
  ],
].filter(Boolean),
```

### 3. Preload Critical Resources

Configure critical resource preloading in `docusaurus.config.js`:

```js
// docusaurus.config.js
module.exports = {
  // ... other config
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossOrigin: 'anonymous',
      },
    },
  ],
};
```

## Troubleshooting Common Deployment Issues

### 1. Base URL Issues

If your site doesn't load properly:

```js
// Make sure baseUrl is correct in docusaurus.config.js
// For username.github.io/repository-name/, use:
baseUrl: '/repository-name/',
```

### 2. Asset Loading Problems

If CSS or JS files don't load:

```js
// Check that trailingSlash is consistent
trailingSlash: false, // or true, but be consistent
```

### 3. 404 Errors

If pages return 404 errors:

- Verify GitHub Pages is enabled in repository settings
- Check that the correct branch is selected (usually `gh-pages`)
- Ensure the `baseUrl` matches your repository name

### 4. Search Not Working

For Algolia search issues:

```js
// Verify Algolia configuration
algolia: {
  appId: process.env.ALGOLIA_APP_ID, // Use environment variables
  apiKey: process.env.ALGOLIA_API_KEY,
  indexName: 'your-index-name',
  contextualSearch: true,
},
```

## Monitoring and Maintenance

### 1. Analytics Setup

Add analytics to track usage:

```js
// docusaurus.config.js
plugins: [
  [
    '@docusaurus/plugin-google-gtag',
    {
      trackingID: 'GA-CODE',
      anonymizeIP: true,
    },
  ],
],
```

### 2. Automated Testing

Set up automated tests for your documentation:

```yaml
# .github/workflows/test.yml
name: Test Documentation

on:
  pull_request:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - name: Link Checker
        run: npx linkinator build/ --recurse --skip "^(?!http://localhost)"
```

## Summary

This chapter covered the complete process of building and deploying Docusaurus sites to GitHub Pages. From local development and testing to automated deployment workflows, you now have all the tools needed to maintain a professional documentation site. The key to successful deployment is proper configuration, thorough testing, and automated processes that ensure consistent, reliable delivery of your technical documentation.