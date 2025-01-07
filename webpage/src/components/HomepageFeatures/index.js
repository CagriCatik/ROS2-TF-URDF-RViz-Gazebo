import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [

  {
    title: 'What is ROS 2?',
    description: (
      <>
        ROS 2 (Robot Operating System 2) is an open-source framework that provides tools and libraries to build and control robots. 
        It introduces modern middleware and communication protocols, making it an ideal choice for robotics development.
      </>
    ),
  },
  {
    title: 'Why Choose ROS 2?',
    description: (
      <>
        ROS 2 offers cross-platform support, real-time capabilities, and improved security compared to its predecessor. 
        It’s designed for both beginners and advanced developers looking to create scalable and robust robotic systems.
      </>
    ),
  },

  {
    title: 'Understanding TFs',
    description: (
      <>
        Learn how to manage coordinate frames using ROS 2 TFs. 
        Understand how transformations help in aligning robot components and integrating sensors, cameras, and robotic arms effectively.
      </>
    ),
  },
  {
    title: 'Design with URDF',
    description: (
      <>
        Master creating and visualizing robot models using URDF. 
        Learn how to define links, joints, and inertial properties for your custom robot designs.
      </>
    ),
  },
  {
    title: 'Visualization with RViz',
    description: (
      <>
        Explore the power of RViz to visualize robot states, sensor data, and TFs in 3D. 
        Create intuitive visual tools to debug and interact with your robots.
      </>
    ),
  },
  {
    title: 'Simulate with Gazebo',
    description: (
      <>
        Dive into simulation with Gazebo. Build dynamic environments to test robot behaviors, physics simulations, and motion algorithms before deploying to real hardware.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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
