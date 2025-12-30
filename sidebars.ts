import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Book Info as top-level documentation entry
  bookInfoSidebar: [
    {
      type: 'category',
      label: 'Book Info',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Module 01: The Robotic Nervous System (ROS 2)',
          collapsible: true,
          collapsed: false,
          link: {
            type: 'doc',
            id: 'module-01/intro', // Link to module intro
          },
          items: [
            'Module_01_The_Robotic_Nervous_System/Chapter_01_Introduction_to_ROS_2/index',
            'Module_01_The_Robotic_Nervous_System/Chapter_02_ROS_2_Nodes_Topics_and_Services/index',
            'Module_01_The_Robotic_Nervous_System/Chapter_03_Building_Python_Agents_with_rclpy/index',
            'Module_01_The_Robotic_Nervous_System/Chapter_04_ROS_2_Services_and_Communication_Patterns/index',
            'Module_01_The_Robotic_Nervous_System/Chapter_05_URDF_Fundamentals_for_Humanoid_Robots/index',
          ],
        },
        {
          type: 'category',
          label: 'Module 02: The Digital Twin (Gazebo & Unity)',
          collapsible: true,
          collapsed: true,
          link: {
            type: 'doc',
            id: 'module-02/intro', // Link to module intro
          },
          items: [
            'module-02/chapter-01-digital-twins',
            'module-02/chapter-02-gazebo-physics',
            'module-02/chapter-03-sensors',
            'module-02/chapter-04-unity',
            'module-02/chapter-05-sim-to-real',
            'module-02/summary',
          ],
        },
        {
          type: 'category',
          label: 'Module 03: The AI-Robot Brain (NVIDIA Isaac™)',
          collapsible: true,
          collapsed: true,
          link: {
            type: 'doc',
            id: 'module-03/intro', // Link to module intro
          },
          items: [
            'module-03/chapter-01-introduction',
            'module-03/chapter-02-isaac-ros',
            'module-03/chapter-03-nav2',
            'module-03/chapter-04-advanced-perception',
            'module-03/chapter-05-capstone',
            'module-03/summary',
          ],
        },
        {
          type: 'category',
          label: 'Module 04: Vision-Language-Action (VLA)',
          collapsible: true,
          collapsed: true,
          link: {
            type: 'doc',
            id: 'module-04/intro', // Link to module intro
          },
          items: [
            'module-04/chapter-01-introduction',
            'module-04/chapter-02-voice-to-action',
            'module-04/chapter-03-cognitive-planning',
            'module-04/chapter-04-simulated-execution',
            'module-04/chapter-05-capstone',
            'module-04/summary',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
