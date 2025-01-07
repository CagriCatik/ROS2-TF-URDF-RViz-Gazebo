# Install and Setup ROS2

## Prerequisites
To follow this part, you need to have Ubuntu 22.04 installed. This section provides a quick recap of the installation process to ensure you’re ready to proceed.

### Recommended Setup
1. Dual Boot: Install Ubuntu alongside your primary OS (e.g., Windows or macOS). This setup ensures seamless compatibility with ROS2 and tools like Gazebo.
2. Virtual Machine (Optional): If dual boot is not feasible, use VMware Workstation. Avoid VirtualBox for this course as it performs poorly with 3D simulation tools like Gazebo.

---

## Step-by-Step Installation of ROS2 Humble

### 1. Install Ubuntu 22.04
Ensure that Ubuntu 22.04 is installed on your system. Dual boot is the recommended method for optimal performance.

### 2. Set Up Locale
Run the following commands to configure the locale for ROS2:
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # Verify settings
```

### 3. Add ROS2 Sources
Enable the ROS2 package repositories by executing the following:
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### 4. Update and Upgrade System
Update your package lists and upgrade installed packages:
```bash
sudo apt update
sudo apt upgrade -y
```

### 5. Install ROS2 Humble
Install the full ROS2 desktop version for tools like RViz and Gazebo:
```bash
sudo apt install -y ros-humble-desktop
```
This installation may download several hundred megabytes and require a few gigabytes of disk space.

### 6. Source ROS2 Setup Script
Configure your environment to source ROS2 automatically:
1. Open your bash configuration file:
   ```bash
   nano ~/.bashrc
   ```
2. Add the following line at the end of the file:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Save and exit, then apply the changes:
   ```bash
   source ~/.bashrc
   ```

### 7. Verify Installation
Run the following command to check if ROS2 is correctly installed:
```bash
ros2 run demo_nodes_cpp talker
```
If the command executes successfully, ROS2 is installed and functional.

---

## Testing Gazebo
Ensure Gazebo runs correctly on your system:
1. Launch Gazebo:
   ```bash
   gazebo
   ```
2. Check the frame rate displayed at the bottom of the Gazebo window:
   - A frame rate of 30 FPS or higher is optimal.
   - If it is below 10 FPS, your system might lack sufficient resources or you are using a virtual machine.

### Troubleshooting
- Low Performance: Switch to a dual boot setup or upgrade your hardware if necessary.
- Unsupported Systems: Avoid running Gazebo on embedded systems like Raspberry Pi.

---

## Hardware Recommendations
To use ROS2 and Gazebo effectively:
- Use a recent computer with adequate performance.
- Avoid running Gazebo on outdated hardware or resource-constrained devices.


