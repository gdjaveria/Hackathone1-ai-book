import type {ReactNode} from 'react';
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
        <Heading as="h1" className="hero__title">
          Physical AI & Humanoid Robotics
        </Heading>
        <p className="hero__subtitle">From digital minds to physical bodies: a journey into embodied intelligence.</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start with the Introduction
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1/introduction">
            Explore Module 1
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const features = [
    {
      title: 'Physical AI & Embodied Intelligence',
      img: '/img/image_1765669349514.webp_image.png',
      description: 'Explore AI systems that can sense, reason, and act in the real world, bridging the gap between digital intelligence and physical interaction.',
    },
    {
      title: 'Simulation to Reality',
      img: '/img/image_1765669562119.webp_image.png',
      description: 'Learn how digital twins and physics-based simulations are used to train and test robots before they are deployed in the real world.',
    },
    {
      title: 'Vision-Language-Action',
      img: '/img/image_1765669705350.webp_image.png',
      description: 'Discover how robots can understand voice commands, use large language models for planning, and execute autonomous behaviors.',
    },
  ];

  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures features={features} />
      </main>
    </Layout>
  );
}
