import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// Same structure and labels as the former mdBook SUMMARY.md. Adding a page: create the
// .mdx file, then list its id (path without extension) here, in reading order.

const sidebars: SidebarsConfig = {
  docs: [
    {type: 'doc', id: 'introduction', label: 'Introduction'},
    {
      type: 'category',
      label: 'Framework',
      collapsed: false,
      items: [
        {type: 'doc', id: 'framework/overview', label: 'Architecture overview'},
        {type: 'doc', id: 'framework/paradigm', label: 'The paradigm: rules and review checklist'},
        {type: 'doc', id: 'framework/scheduler', label: 'The scheduler'},
        {type: 'doc', id: 'framework/messages', label: 'Messages and data types'},
        {type: 'doc', id: 'framework/communications', label: 'Communications'},
        {type: 'doc', id: 'framework/conventions', label: 'Conventions and invariants'},
      ],
    },
    {
      type: 'category',
      label: 'Mathematical foundations',
      collapsed: false,
      items: [
        {type: 'doc', id: 'math/frames-and-rotations', label: 'Frames, vectors and rotations'},
        {type: 'doc', id: 'math/rigid-body', label: 'Rigid-body efforts: wrench, mass and inertia'},
        {type: 'doc', id: 'math/cascaded-control', label: 'Cascaded control'},
        {type: 'doc', id: 'math/least-squares', label: 'Constrained least squares'},
      ],
    },
    {
      type: 'category',
      label: 'Robotics pipeline: motor control',
      collapsed: false,
      items: [
        {type: 'doc', id: 'pipelines/motor-control/overview', label: 'Overview'},
        {type: 'doc', id: 'pipelines/motor-control/motor-model', label: 'The motor model'},
        {type: 'doc', id: 'pipelines/motor-control/mixer', label: 'The mixer: control allocation'},
        {type: 'doc', id: 'pipelines/motor-control/attitude-controller', label: 'The attitude controller'},
        {type: 'doc', id: 'pipelines/motor-control/resolver', label: 'The force → attitude resolver'},
        {type: 'doc', id: 'pipelines/motor-control/roadmap', label: 'Roadmap: the remaining stages'},
        {type: 'doc', id: 'pipelines/motor-control/validation', label: 'Validation'},
        {type: 'doc', id: 'pipelines/motor-control/lessons', label: 'Lessons learned'},
      ],
    },
    {
      type: 'category',
      label: 'Simulation',
      collapsed: false,
      items: [{type: 'doc', id: 'simulation/gazebo', label: 'The Gazebo bridge'}],
    },
    {type: 'doc', id: 'glossary', label: 'Glossary'},
    {type: 'doc', id: 'contributing', label: 'Extending this documentation'},
  ],
};

export default sidebars;
