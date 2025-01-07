"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[6544],{7618:(e,s,n)=>{n.r(s),n.d(s,{assets:()=>a,contentTitle:()=>i,default:()=>u,frontMatter:()=>o,metadata:()=>r,toc:()=>d});const r=JSON.parse('{"id":"intro/install_setup_ros2","title":"Install and Setup ROS2","description":"Prerequisites","source":"@site/docs/01_intro/02_install_setup_ros2.md","sourceDirName":"01_intro","slug":"/intro/install_setup_ros2","permalink":"/ROS2-TF-URDF-RViz-Gazebo/docs/intro/install_setup_ros2","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/01_intro/02_install_setup_ros2.md","tags":[],"version":"current","sidebarPosition":2,"frontMatter":{},"sidebar":"introSidebar","previous":{"title":"Overview","permalink":"/ROS2-TF-URDF-RViz-Gazebo/docs/intro/overview"},"next":{"title":"Programming Tools for ROS 2","permalink":"/ROS2-TF-URDF-RViz-Gazebo/docs/intro/programming_tools"}}');var t=n(4848),l=n(8453);const o={},i="Install and Setup ROS2",a={},d=[{value:"Prerequisites",id:"prerequisites",level:2},{value:"Recommended Setup",id:"recommended-setup",level:3},{value:"Step-by-Step Installation of ROS2 Humble",id:"step-by-step-installation-of-ros2-humble",level:2},{value:"1. Install Ubuntu 22.04",id:"1-install-ubuntu-2204",level:3},{value:"2. Set Up Locale",id:"2-set-up-locale",level:3},{value:"3. Add ROS2 Sources",id:"3-add-ros2-sources",level:3},{value:"4. Update and Upgrade System",id:"4-update-and-upgrade-system",level:3},{value:"5. Install ROS2 Humble",id:"5-install-ros2-humble",level:3},{value:"6. Source ROS2 Setup Script",id:"6-source-ros2-setup-script",level:3},{value:"7. Verify Installation",id:"7-verify-installation",level:3},{value:"Testing Gazebo",id:"testing-gazebo",level:2},{value:"Troubleshooting",id:"troubleshooting",level:3},{value:"Hardware Recommendations",id:"hardware-recommendations",level:2}];function c(e){const s={code:"code",h1:"h1",h2:"h2",h3:"h3",header:"header",hr:"hr",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",ul:"ul",...(0,l.R)(),...e.components};return(0,t.jsxs)(t.Fragment,{children:[(0,t.jsx)(s.header,{children:(0,t.jsx)(s.h1,{id:"install-and-setup-ros2",children:"Install and Setup ROS2"})}),"\n",(0,t.jsx)(s.h2,{id:"prerequisites",children:"Prerequisites"}),"\n",(0,t.jsxs)(s.p,{children:["To follow this part, you need to have ",(0,t.jsx)(s.strong,{children:"Ubuntu 22.04"})," installed. This section provides a quick recap of the installation process to ensure you\u2019re ready to proceed."]}),"\n",(0,t.jsx)(s.h3,{id:"recommended-setup",children:"Recommended Setup"}),"\n",(0,t.jsxs)(s.ol,{children:["\n",(0,t.jsxs)(s.li,{children:[(0,t.jsx)(s.strong,{children:"Dual Boot"}),": Install Ubuntu alongside your primary OS (e.g., Windows or macOS). This setup ensures seamless compatibility with ROS2 and tools like Gazebo."]}),"\n",(0,t.jsxs)(s.li,{children:[(0,t.jsx)(s.strong,{children:"Virtual Machine (Optional)"}),": If dual boot is not feasible, use ",(0,t.jsx)(s.strong,{children:"VMware Workstation"}),". Avoid VirtualBox for this course as it performs poorly with 3D simulation tools like Gazebo."]}),"\n"]}),"\n",(0,t.jsx)(s.hr,{}),"\n",(0,t.jsx)(s.h2,{id:"step-by-step-installation-of-ros2-humble",children:"Step-by-Step Installation of ROS2 Humble"}),"\n",(0,t.jsx)(s.h3,{id:"1-install-ubuntu-2204",children:"1. Install Ubuntu 22.04"}),"\n",(0,t.jsx)(s.p,{children:"Ensure that Ubuntu 22.04 is installed on your system. Dual boot is the recommended method for optimal performance."}),"\n",(0,t.jsx)(s.h3,{id:"2-set-up-locale",children:"2. Set Up Locale"}),"\n",(0,t.jsx)(s.p,{children:"Run the following commands to configure the locale for ROS2:"}),"\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"sudo locale-gen en_US en_US.UTF-8\nsudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8\nexport LANG=en_US.UTF-8\nlocale  # Verify settings\n"})}),"\n",(0,t.jsx)(s.h3,{id:"3-add-ros2-sources",children:"3. Add ROS2 Sources"}),"\n",(0,t.jsx)(s.p,{children:"Enable the ROS2 package repositories by executing the following:"}),"\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"sudo apt update\nsudo apt install -y software-properties-common\nsudo add-apt-repository universe\nsudo apt update\nsudo apt install -y curl gnupg lsb-release\ncurl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -\nsudo sh -c 'echo \"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2-latest.list'\n"})}),"\n",(0,t.jsx)(s.h3,{id:"4-update-and-upgrade-system",children:"4. Update and Upgrade System"}),"\n",(0,t.jsx)(s.p,{children:"Update your package lists and upgrade installed packages:"}),"\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"sudo apt update\nsudo apt upgrade -y\n"})}),"\n",(0,t.jsx)(s.h3,{id:"5-install-ros2-humble",children:"5. Install ROS2 Humble"}),"\n",(0,t.jsx)(s.p,{children:"Install the full ROS2 desktop version for tools like RViz and Gazebo:"}),"\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"sudo apt install -y ros-humble-desktop\n"})}),"\n",(0,t.jsx)(s.p,{children:"This installation may download several hundred megabytes and require a few gigabytes of disk space."}),"\n",(0,t.jsx)(s.h3,{id:"6-source-ros2-setup-script",children:"6. Source ROS2 Setup Script"}),"\n",(0,t.jsx)(s.p,{children:"Configure your environment to source ROS2 automatically:"}),"\n",(0,t.jsxs)(s.ol,{children:["\n",(0,t.jsxs)(s.li,{children:["Open your bash configuration file:","\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"nano ~/.bashrc\n"})}),"\n"]}),"\n",(0,t.jsxs)(s.li,{children:["Add the following line at the end of the file:","\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"source /opt/ros/humble/setup.bash\n"})}),"\n"]}),"\n",(0,t.jsxs)(s.li,{children:["Save and exit, then apply the changes:","\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"source ~/.bashrc\n"})}),"\n"]}),"\n"]}),"\n",(0,t.jsx)(s.h3,{id:"7-verify-installation",children:"7. Verify Installation"}),"\n",(0,t.jsx)(s.p,{children:"Run the following command to check if ROS2 is correctly installed:"}),"\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"ros2 run demo_nodes_cpp talker\n"})}),"\n",(0,t.jsx)(s.p,{children:"If the command executes successfully, ROS2 is installed and functional."}),"\n",(0,t.jsx)(s.hr,{}),"\n",(0,t.jsx)(s.h2,{id:"testing-gazebo",children:"Testing Gazebo"}),"\n",(0,t.jsx)(s.p,{children:"Ensure Gazebo runs correctly on your system:"}),"\n",(0,t.jsxs)(s.ol,{children:["\n",(0,t.jsxs)(s.li,{children:["Launch Gazebo:","\n",(0,t.jsx)(s.pre,{children:(0,t.jsx)(s.code,{className:"language-bash",children:"gazebo\n"})}),"\n"]}),"\n",(0,t.jsxs)(s.li,{children:["Check the frame rate displayed at the bottom of the Gazebo window:","\n",(0,t.jsxs)(s.ul,{children:["\n",(0,t.jsx)(s.li,{children:"A frame rate of 30 FPS or higher is optimal."}),"\n",(0,t.jsx)(s.li,{children:"If it is below 10 FPS, your system might lack sufficient resources or you are using a virtual machine."}),"\n"]}),"\n"]}),"\n"]}),"\n",(0,t.jsx)(s.h3,{id:"troubleshooting",children:"Troubleshooting"}),"\n",(0,t.jsxs)(s.ul,{children:["\n",(0,t.jsxs)(s.li,{children:[(0,t.jsx)(s.strong,{children:"Low Performance"}),": Switch to a dual boot setup or upgrade your hardware if necessary."]}),"\n",(0,t.jsxs)(s.li,{children:[(0,t.jsx)(s.strong,{children:"Unsupported Systems"}),": Avoid running Gazebo on embedded systems like Raspberry Pi."]}),"\n"]}),"\n",(0,t.jsx)(s.hr,{}),"\n",(0,t.jsx)(s.h2,{id:"hardware-recommendations",children:"Hardware Recommendations"}),"\n",(0,t.jsx)(s.p,{children:"To use ROS2 and Gazebo effectively:"}),"\n",(0,t.jsxs)(s.ul,{children:["\n",(0,t.jsx)(s.li,{children:"Use a recent computer with adequate performance."}),"\n",(0,t.jsx)(s.li,{children:"Avoid running Gazebo on outdated hardware or resource-constrained devices."}),"\n"]})]})}function u(e={}){const{wrapper:s}={...(0,l.R)(),...e.components};return s?(0,t.jsx)(s,{...e,children:(0,t.jsx)(c,{...e})}):c(e)}},8453:(e,s,n)=>{n.d(s,{R:()=>o,x:()=>i});var r=n(6540);const t={},l=r.createContext(t);function o(e){const s=r.useContext(l);return r.useMemo((function(){return"function"==typeof e?e(s):{...s,...e}}),[s,e])}function i(e){let s;return s=e.disableParentContext?"function"==typeof e.components?e.components(t):e.components||t:o(e.components),r.createElement(l.Provider,{value:s},e.children)}}}]);