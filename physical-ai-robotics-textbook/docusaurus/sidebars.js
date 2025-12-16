const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: '00 - Introduction',
      items: [
        'introduction/index',
        'introduction/welcome',
        'introduction/prerequisites',
        'introduction/hardware-requirements',
        'introduction/how-to-use',
        'introduction/syllabus',
      ],
    },
    {
      type: 'category',
      label: '01 - ROS 2 Architecture & Concepts',
      items: [
        'ros2/index',
        'ros2/01-ros2-architecture',
        'ros2/01-ros2-nodes-topics',
        'ros2/01-ros2-services-actions',
        'ros2/python-packages',
        'ros2/urdf-humanoids',
        'ros2/launch-files',
      ],
    },
    {
      type: 'category',
      label: '02 - Simulation with Gazebo',
      items: [
        'simulation/index',
        'simulation/gazebo-intro',
        'simulation/urdf-sdf',
        'simulation/sensors-plugins',
        'simulation/world-building',
        'simulation/ros2-integration',
        'simulation/advanced-simulation',
      ],
    },
    {
      type: 'category',
      label: '03 - NVIDIA Isaac Sim',
      items: [
        'isaac/index',
        'isaac/isaac-sim',
        'isaac/isaac-ros',
        'isaac/vslam-navigation',
        'isaac/perception',
        'isaac/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: '04 - Vision-Language-Action (VLA)',
      items: [
        'vla/index',
        'vla/voice-to-action',
        'vla/llm-planning',
        'vla/natural-language',
        'vla/multimodal',
      ],
    },
    {
      type: 'category',
      label: '05 - Capstone Project',
      items: [
        'capstone/index',
        'capstone/project-overview',
        'capstone/architecture',
        'capstone/voice-system',
        'capstone/navigation',
        'capstone/manipulation',
        'capstone/integration',
      ],
    },
  ],
};

module.exports = sidebars;