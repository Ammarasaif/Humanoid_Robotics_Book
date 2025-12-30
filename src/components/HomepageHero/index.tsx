import React, { useState, useEffect } from 'react';
import { Engine } from '@tsparticles/engine';
import { initParticlesEngine } from '@tsparticles/react';
import { loadSlim } from '@tsparticles/slim'; // Using slim version for lightweight implementation
import Particles from '@tsparticles/react';
import styles from './styles.module.css';

const HomepageHero = () => {
  const [init, setInit] = useState(false);
  const [textVisible, setTextVisible] = useState(false);

  // Initialize particles engine
  useEffect(() => {
    initParticlesEngine(async (engine: Engine) => {
      await loadSlim(engine);
    }).then(() => {
      setInit(true);
    });
  }, []);

  // Trigger text fade-in animation on component mount
  useEffect(() => {
    setTextVisible(true);
  }, []);

  // Particle configuration for subtle floating effect
  const particlesLoaded = async (container: any) => {
    // Optional: Perform actions when particles are loaded
  };

  const particlesOptions = {
    background: {
      color: {
        value: "transparent",
      },
    },
    fpsLimit: 120,
    particles: {
      color: {
        value: "#ffffff",
        animation: {
          enable: true,
          speed: 20,
          sync: false
        },
      },
      links: {
        enable: false,
      },
      move: {
        enable: true,
        speed: 0.5, // Slow movement for subtle effect
        direction: "none",
        random: true,
        straight: false,
        outModes: {
          default: "out",
        },
        trail: {
          enable: false,
        },
      },
      number: {
        density: {
          enable: true,
          area: 800,
        },
        value: 40, // Moderate number of particles for performance
      },
      opacity: {
        value: 0.5,
        animation: {
          enable: true,
          speed: 3,
          minimumValue: 0.1,
          sync: false,
        },
      },
      shape: {
        type: "circle", // Simple circles for lightweight rendering
      },
      size: {
        value: { min: 1, max: 3 }, // Small particles to not interfere with text
      },
      twinkle: {
        lines: {
          enable: false,
        },
        particles: {
          enable: true,
          frequency: 0.05,
          opacity: 0.5,
        },
      },
    },
    detectRetina: true,
  };

  return (
    <section className={styles.heroSection}>
      {/* Particle background effect - appears behind overlay but above background image */}
      {init && (
        <Particles
          id="tsparticles"
          particlesLoaded={particlesLoaded}
          options={particlesOptions}
          className={styles.particleCanvas}
        />
      )}
      <div className={styles.heroOverlay} />
      <div className={`${styles.heroContent} ${textVisible ? styles.textFadeIn : ''}`}>
        <h1 className={`${styles.heroTitle} ${textVisible ? styles.textFadeIn : ''}`}>
          Physical AI & Humanoid Robotics
        </h1>
        <p className={`${styles.heroSubtitle} ${textVisible ? styles.textFadeIn : ''}`}>
          Embodied Intelligence — Bridging Digital Brains with Physical Bodies
        </p>
        <a href="/docs/module-01/intro" className={styles.ctaButton} aria-label="Start Reading">
          Start Reading
        </a>
      </div>
    </section>
  );
};

export default HomepageHero;