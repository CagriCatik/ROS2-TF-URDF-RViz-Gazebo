// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  introSidebar: [{type: 'autogenerated', dirName: '01_intro'}],
  transformSidebar: [{type: 'autogenerated', dirName: '02_transform_overview'}],
  urdfSidebar: [{type: 'autogenerated', dirName: '03_urdf_robot'}],
  broadcastSidebar: [{type: 'autogenerated', dirName: '04_broadcast_TF'}],
  urdfXacroSidebar: [{type: 'autogenerated', dirName: '05_urdf_xacro'}],
  gazeboXacroSidebar: [{type: 'autogenerated', dirName: '06_simulation_gazebo'}],
  sensorXacroSidebar: [{type: 'autogenerated', dirName: '07_sensors'}],
};

export default sidebars;