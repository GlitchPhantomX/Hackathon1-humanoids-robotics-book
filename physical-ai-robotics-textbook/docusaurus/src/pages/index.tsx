import React from 'react';
import clsx from 'clsx';
import { Link } from 'lucide-react';
// @ts-ignore - Docusaurus provides this module at runtime, but TypeScript may not see its types
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
// @ts-ignore - provided by Docusaurus theme at runtime; types are supplied by the framework
import Layout from '@theme/Layout';
// @ts-ignore - resolved via Docusaurus site alias; TS doesn't see the module path
// @ts-ignore - CSS modules are handled by the bundler, not directly by TypeScript
import styles from './index.module.css';
import '@site/src/styles/chatbot.css';
import Banner from '../components/Banner';
import HeroSection from '../components/HeroSection';
import PhysicalAISection from '../components/PhysicalAISection';
import PhysicalAIPillars from "../components/PhysicalAIPillars"
import Footer from '../components/Footer';
import LearningJourney from '../components/LearningJourney/LearningJourney';
import PhysicalAIParadigmShift from '../components/PhysicalAIParadigmShift';
import PhysicalAICTA from '../components/PhysicalAICTA';
import ThreePoints from '../components/ThreePoints';
// import ChatWidget from '../components/ChatWidget'; // Import ChatWidget


function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {  // `: JSX.Element` hata diya
  const {siteConfig} = useDocusaurusContext();
  return (
  <div className="">
   <Banner/>
      <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn about Physical AI and Humanoid Robotics">
      {/* <HomepageHeader /> */}
      <main>
         {/* <div className="docs-chatbot-sidebar">
          <ChatWidget />
        </div>  */}
      
        <HeroSection/>
        <PhysicalAISection/>
        <ThreePoints/>
        <PhysicalAIPillars/>
        <LearningJourney/>
        <PhysicalAIParadigmShift/>
        <PhysicalAICTA/>
        <Footer/>
      </main>

    </Layout>
  </div>
  );
}