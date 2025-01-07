# Creating and Visualizing a Simple URDF Link

## Introduction

In this lesson, you will learn how to create a URDF (Unified Robot Description Format) file, add your first link to define a robot's structure, and visualize the URDF using RViz. This hands-on approach will provide you with a foundational understanding of how to describe and visualize robotic components within the ROS2 framework.

## Prerequisites

Before proceeding, ensure that you have the following:

- **ROS2 Installed and Configured**: Make sure ROS2 is properly installed on your system and that your environment is sourced.
- **URDF Tutorial Package Installed**: Familiarity with the `urdf_tutorial` package is beneficial for visualization purposes.
- **Visual Studio Code (VS Code)**: Installed as your preferred text editor for editing URDF files.
- **Basic Understanding of XML**: Since URDF files are XML-based, understanding XML syntax will be advantageous.

## Step 1: Creating the URDF File

### 1.1. Open a Terminal

Begin by launching a terminal window where you will create and edit your URDF file.

### 1.2. Create the URDF File

Navigate to your home directory or a preferred workspace directory and create a new URDF file named `my_robot.urdf`.

```bash
cd ~
touch my_robot.urdf
```

### 1.3. Edit the URDF File

Open the newly created URDF file using Visual Studio Code.

```bash
code my_robot.urdf
```

## Step 2: Writing the URDF Structure

### 2.1. Initialize the XML Structure

URDF files follow XML syntax. Start by defining the XML declaration and the root `<robot>` tag with a name attribute.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
</robot>
```

### 2.2. Add the First Link

Within the `<robot>` tag, define your first link, typically named `base_link`. This link will serve as the chassis of your robot.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <!-- Base Link -->
    <link name="base_link">
    </link>
</robot>
```

## Step 3: Defining the Link's Visual Properties

### 3.1. Add Visual Geometry

To give shape to the `base_link`, add a `<visual>` tag containing a `<geometry>` tag. Here, we'll define the base link as a box.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
        </visual>
    </link>
</robot>
```

**Explanation:**

- `<box size="0.6 0.4 0.2"/>`: Defines a box with length 0.6 meters, width 0.4 meters, and height 0.2 meters.

### 3.2. Specify Material Properties

Enhance the visual appearance by specifying the material and color of the base link.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
    </link>
</robot>
```

**Explanation:**

- `<material name="blue">`: Assigns the color blue to the base link.
- `<color rgba="0 0 1 1"/>`: Specifies the RGBA color values (red, green, blue, alpha).

### 3.3. Define the Origin of the Visual

By default, the visual geometry is centered at the link's origin. To position the base link above the ground, adjust the origin using the `<origin>` tag.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
        </visual>
    </link>
</robot>
```

**Explanation:**

- `<origin xyz="0 0 0.1" rpy="0 0 0"/>`: Offsets the visual geometry by 0.1 meters along the Z-axis, positioning the base link above the ground.

### 3.4. Complete URDF File

Your complete `my_robot.urdf` file should now look like this:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
        </visual>
    </link>
</robot>
```

## Step 4: Visualizing the URDF in RViz

### 4.1. Launch the URDF Tutorial Package

Ensure that the `urdf_tutorial` package is installed. If not, install it using the following command (replace `<ros-distro>` with your ROS2 distribution, e.g., `humble`):

```bash
sudo apt update
sudo apt install ros-<ros-distro>-urdf-tutorial
```

For ROS2 Humble:

```bash
sudo apt install ros-humble-urdf-tutorial
```

### 4.2. Source the ROS2 Environment

After installation, source the ROS2 setup script to configure environment variables:

```bash
source /opt/ros/<ros-distro>/setup.bash
```

For ROS2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

To automate this process, add the above line to your `.bashrc` file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4.3. Launch the Visualization

Use the `urdf_tutorial` launch file to visualize your URDF in RViz. Navigate to the directory containing your `my_robot.urdf` file.

```bash
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

**Explanation:**

- `display.launch.py`: Launch file provided by the `urdf_tutorial` package for visualization.
- `model:=my_robot.urdf`: Specifies the URDF file to visualize.

### 4.4. Understanding the Launch Process

Upon executing the launch command, you will see log messages indicating the initiation of various nodes, including the `joint_state_publisher` and `robot_state_publisher`. These nodes are responsible for publishing the robot's state and transforms, respectively.

Example output:

```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2025-01-07-12-34-56-789012-user-ros2-launch-0.log
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [joint_state_publisher]: Starting joint state publisher node
[INFO] [robot_state_publisher]: Starting robot state publisher node
[INFO] [rviz2]: Starting RViz
```

### 4.5. Exploring the Visualization in RViz

Once RViz launches, you will see a 3D representation of your robot along with its transform frames.

#### 4.5.1. Navigating the RViz Interface

- **3D Viewport**: The central area displays the robot model. Use mouse controls to rotate, zoom, and pan the view.
- **Displays Panel**: Located on the left, this panel lists all active visualization elements such as the robot model, TF frames, and grid.



```
![RViz Interface](images/rviz_interface.png)
```


```
*Note: Replace the image placeholder with an actual screenshot of the RViz interface showing the robot model and TF frames.*

#### 4.5.2. Inspecting the Robot Model

- **Robot Model**: Displays the visual geometry defined in your URDF (`base_link` as a blue box).
- **TF Frames**: Represented by colored axes (X in red, Y in green, Z in blue) indicating the orientation and position of each link.
```
```
![Robot Model in RViz](images/robot_model_rviz.png)
```
```
*Note: Replace the image placeholder with an actual screenshot of the robot model in RViz.*

### 4.6. Adjusting the Visual Origin

By default, the visual geometry (`base_link`) is centered at the origin. To position the base link correctly on the ground, adjust the origin in the URDF file.

#### 4.6.1. Offset the Visual Geometry

Edit the `<origin>` tag within the `<visual>` section to offset the base link by half its height along the Z-axis.

```xml
<origin xyz="0 0 0.1" rpy="0 0 0"/>
```

**Explanation:**

- `xyz="0 0 0.1"`: Moves the visual geometry 0.1 meters upward along the Z-axis, placing the base link on the ground.

#### 4.6.2. Save and Refresh

After making changes, save the `my_robot.urdf` file. RViz should automatically update the visualization to reflect the changes. If not, restart the launch command.

```bash
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

```
![Adjusted Visual Origin](images/adjusted_visual_origin.png)
```
*Note: Replace the image placeholder with an actual screenshot showing the base link positioned on the ground.*

## Step 5: Enhancing the URDF with Additional Properties

### 5.1. Adding Collision Properties

To enable collision detection, define the `<collision>` tag within the `<link>`.

```xml
<link name="base_link">
    <visual>
        <!-- Visual properties -->
    </visual>
    <collision>
        <geometry>
            <box size="0.6 0.4 0.2"/>
        </geometry>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
</link>
```

**Explanation:**

- `<collision>`: Defines the geometry used for collision detection, which can differ from the visual geometry.

### 5.2. Specifying Inertial Properties

Inertial properties are essential for simulation purposes, defining mass and inertia tensors.

```xml
<link name="base_link">
    <visual>
        <!-- Visual properties -->
    </visual>
    <collision>
        <!-- Collision properties -->
    </collision>
    <inertial>
        <mass value="10.0"/>
        <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
</link>
```

**Explanation:**

- `<mass value="10.0"/>`: Assigns a mass of 10 kilograms to the base link.
- `<inertia>`: Defines the inertia tensor components (`ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz`).

## Step 6: Testing and Iterating

### 6.1. Validate in RViz

After defining all necessary properties, launch the visualization again to ensure that the robot appears as expected.

```bash
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

### 6.2. Troubleshooting

- **No Robot Displayed**: Ensure that the URDF syntax is correct and that all tags are properly closed.
- **Incorrect Dimensions**: Verify the size parameters in the `<geometry>` tags.
- **Visualization Issues**: Check the `<origin>` offsets and ensure that the visual geometry is positioned correctly.

### 6.3. Iterative Development

Continue adding more links and joints to build a more complex robot. Regularly visualize in RViz to confirm each addition behaves as intended.

## Conclusion

Creating and visualizing a simple URDF link is the first step toward building comprehensive robotic models in ROS2. By defining links and their visual properties, and by leveraging RViz for visualization, you can iteratively develop and refine your robot's structure. This foundational knowledge sets the stage for more advanced URDF features, such as defining multiple joints, sensors, and dynamic components, enabling the creation of sophisticated and autonomous robotic systems.

---

## Additional Resources

- **URDF Tutorials**: [ROS2 URDF Tutorials](https://docs.ros.org/en/humble/Tutorials.html#robot-description-formats)
- **Xacro Documentation**: [Xacro Overview](https://wiki.ros.org/xacro)
- **RViz Documentation**: [RViz User Guide](https://docs.ros.org/en/humble/Tutorials/VisualizingRviz.html)

---

By following this guide and utilizing the provided examples, you will gain a solid understanding of how to create and manage URDF files, enabling you to effectively describe and visualize your robotic systems within ROS2.