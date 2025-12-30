import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import autonomousSystem from '@site/static/img/autonomous_system.jpg';
import cognitiveEmbodiment from '@site/static/img/cognitive Embodiment.jpeg';
import realWorldIntelligence from '@site/static/img/real_world_intelligence.webp';

type FeatureItem = {
  title: string;
  imageUrl: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Autonomous Systems',
    imageUrl: autonomousSystem,
    description: (
      <>
        Intelligent agents capable of independent perception, decision-making, and action in complex real-world environments.
      </>
    ),
  },
  {
    title: 'Cognitive Embodiment',
    imageUrl: cognitiveEmbodiment,
    description: (
      <>
        Human-inspired systems integrating physical bodies with cognitive processes to learn, adapt, and interact naturally.
      </>
    ),
  },
  {
    title: 'Real-World Intelligence',
    imageUrl: realWorldIntelligence,
    description: (
      <>
        AI systems designed for practical tasks, seamlessly bridging perception, reasoning, and actuation in dynamic settings.
      </>
    ),
  },
];

function Feature({title, imageUrl, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img src={imageUrl} className={styles.featureImage} role="img" alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
