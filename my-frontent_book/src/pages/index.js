import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.primaryButton)}
              to="/docs/intro">
              Get Started
            </Link>
            <Link
              className={clsx('button button--secondary button--lg', styles.secondaryButton)}
              to="/docs/ros2-module/intro">
              Learn More
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function ImageContentSection() {
  return (
    <section className={styles.imageContentSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>Beautiful Content with Images</Heading>
          <p className={styles.sectionSubtitle}>Explore our stunning visual content with modern design</p>
        </div>
        <div className={styles.imageGrid}>
          <div className={styles.imageCard}>
            <div className={styles.imagePlaceholder}>🖼️</div>
            <div className={styles.imageCardContent}>
              <Heading as="h3" className={styles.imageCardTitle}>Modern Design</Heading>
              <p className={styles.imageCardDescription}>Beautiful interfaces crafted with attention to detail</p>
            </div>
          </div>
          <div className={styles.imageCard}>
            <div className={styles.imagePlaceholder}>🎨</div>
            <div className={styles.imageCardContent}>
              <Heading as="h3" className={styles.imageCardTitle}>Creative Solutions</Heading>
              <p className={styles.imageCardDescription}>Innovative approaches to complex problems</p>
            </div>
          </div>
          <div className={styles.imageCard}>
            <div className={styles.imagePlaceholder}>🚀</div>
            <div className={styles.imageCardContent}>
              <Heading as="h3" className={styles.imageCardTitle}>Fast Performance</Heading>
              <p className={styles.imageCardDescription}>Optimized for speed and efficiency</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <ImageContentSection />
      </main>
    </Layout>
  );
}
