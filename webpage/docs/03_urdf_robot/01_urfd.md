# Introduction to URDF

In the previous section, we introduced **TF (Transform)** and emphasized its importance in running robotic systems with ROS2. We also noted that to generate TF effectively, it is necessary to describe the robot’s structure in a specific format. This is where **URDF** comes into play.

---

## What is URDF?

**URDF** stands for **Unified Robot Description Format**. It is an XML-based file format used to describe the physical structure of a robot. The URDF file specifies:

1. **Links**: The physical, rigid components of the robot.
2. **Joints**: The connections between links that define their relationships and allow motion.

By defining these elements, a URDF file provides a complete description of the robot's structure, enabling ROS2 to generate the necessary transforms for TF and perform other critical tasks.

### Key Features of URDF

- **Robot Description**: Encodes the robot’s physical structure, including dimensions, mass, and material properties.
- **Visualization**: Allows you to view the robot’s model in tools like **RViz** for validation and debugging.
- **Simulation**: Enables the simulation of the robot in environments like **Gazebo**.
- **Compatibility with TF**: Automatically generates transforms between coordinate frames based on the defined links and joints.

---

## Anatomy of a URDF File

A URDF file is composed of XML elements that define the various components and their relationships within the robot. Understanding the basic structure of a URDF file is essential for effectively describing and visualizing robotic systems.

### Links

**Links** represent the rigid parts of the robot. Each link is defined with:

- **Name**: A unique identifier for the link.
- **Visual Properties**: Meshes or geometric shapes to represent the link visually.
- **Collision Properties**: Geometric shapes used for collision detection.
- **Inertial Properties**: Mass and inertia tensor for simulation purposes.

#### Example: Defining a Link

```xml
<!-- Definition of the base_link -->
<link name="base_link">
    <visual>
        <geometry>
            <box size="1 1 0.5"/>
        </geometry>
        <material name="blue">
            <color rgba="0 0 1 1"/>
        </material>
    </visual>
    <collision>
        <geometry>
            <box size="1 1 0.5"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="5.0"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
</link>
```

### Joints

**Joints** define the relationships between links, including:

- **Parent and Child Links**: Specifies which two links the joint connects.
- **Type**: Fixed, revolute, prismatic, etc., to define the motion allowed by the joint.
- **Origin**: Position and orientation of the joint relative to the parent link.

#### Example: Defining a Revolute Joint

```xml
<!-- Definition of a revolute joint connecting base_link to wheel_link -->
<joint name="base_to_wheel" type="revolute">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
</joint>
```

### XML Syntax

URDF files are written in XML, with a hierarchical structure that organizes links and joints. Here is a basic example:

```xml
<robot name="simple_robot">
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 0.5"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
    </link>

    <!-- Wheel Link -->
    <link name="wheel_link">
        <visual>
            <geometry>
                <cylinder radius="0.2" length="0.1"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
    </link>

    <!-- Revolute Joint -->
    <joint name="base_to_wheel" type="revolute">
        <parent link="base_link"/>
        <child link="wheel_link"/>
        <origin xyz="0.5 0 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>
</robot>
```

---

## Hands-On: Building a Simple Robot URDF

In this section, we will create a URDF file for a mobile robot with two wheels.

### Step 1: Define the Links

1. **Base Link**:
   - The central body of the robot.
   - Represented as a rectangular box.

2. **Wheel Links**:
   - Two wheels, each represented as a cylinder.

#### Example: Defining Links

```xml
<!-- mobile_robot.urdf -->
<robot name="mobile_robot">
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 0.5"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="1 1 0.5"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
        </inertial>
    </link>

    <!-- Left Wheel Link -->
    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.2" length="0.1"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.2" length="0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
        </inertial>
    </link>

    <!-- Right Wheel Link -->
    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.2" length="0.1"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.2" length="0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
        </inertial>
    </link>
</robot>
```

### Step 2: Define the Joints

1. **Base to Wheels**:
   - Connects the base link to each wheel.
   - Defined as a revolute joint to allow rotational motion.

#### Example: Defining Joints

```xml
<!-- Revolute Joint for Left Wheel -->
<joint name="base_to_left_wheel" type="revolute">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.5 0.5 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="1.0" lower="-3.14" upper="3.14"/>
</joint>

<!-- Revolute Joint for Right Wheel -->
<joint name="base_to_right_wheel" type="revolute">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.5 -0.5 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="1.0" lower="-3.14" upper="3.14"/>
</joint>
```

### Step 3: Complete URDF File

Combine the links and joints to form the complete URDF file for the mobile robot.

```xml
<!-- mobile_robot.urdf -->
<robot name="mobile_robot">
    <!-- Links -->
    <!-- (Include the link definitions from Step 1 here) -->

    <!-- Joints -->
    <!-- (Include the joint definitions from Step 2 here) -->
</robot>
```

### Step 4: Visualize in RViz

Using the `robot_state_publisher` node, you can visualize the URDF in RViz to ensure the structure is correctly defined.

#### Example: Launching the Visualization

1. **Create a Launch File**

   ```python
   # mobile_robot_launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': open('/path/to/mobile_robot.urdf').read()}]
           ),
           Node(
               package='rviz2',
               executable='rviz2',
               name='rviz2',
               output='screen',
               arguments=['-d', '/path/to/rviz_config.rviz']
           ),
       ])
   ```

   Replace `/path/to/mobile_robot.urdf` with the actual path to your URDF file and `/path/to/rviz_config.rviz` with the path to your RViz configuration file.

2. **Execute the Launch File**

   ```bash
   ros2 launch /path/to/mobile_robot_launch.py
   ```

   This command will open RViz and display the mobile robot along with its transform frames.

---

## Best Practices for URDF

Creating an effective URDF file involves adhering to certain best practices to ensure maintainability, scalability, and accuracy.

1. **Organized Structure**: Maintain a clear hierarchy of links and joints to reflect the physical structure of the robot accurately.
   
   ```xml
   <!-- Organized Hierarchy Example -->
   <robot name="organized_robot">
       <link name="base_link">
           <!-- Base link properties -->
       </link>
       <joint name="base_to_arm" type="revolute">
           <!-- Joint properties -->
       </joint>
       <link name="arm_link">
           <!-- Arm link properties -->
       </link>
       <!-- Additional links and joints -->
   </robot>
   ```

2. **Validation**: Visualize in RViz frequently to ensure correctness and catch errors early in the development process.
   
   ```bash
   # Example: Visualizing URDF in RViz
   ros2 launch urdf_tutorial display.launch.py model:=mobile_robot
   ```

3. **Reusability**: Use modular designs by creating separate URDF files for different robot components (e.g., arms, sensors) and include them using the `<xacro:include>` directive or similar mechanisms.
   
   ```xml
   <!-- Including a separate URDF file for the arm -->
   <xacro:include filename="arm.urdf.xacro"/>
   <xacro:arm/>
   ```

4. **Comments**: Add comments in your URDF to document key decisions and parameters, enhancing readability and maintainability.
   
   ```xml
   <!-- Revolute joint connecting base_link to left_wheel -->
   <joint name="base_to_left_wheel" type="revolute">
       <!-- Joint properties -->
   </joint>
   ```

5. **Consistent Naming Conventions**: Adopt a consistent and descriptive naming convention for links and joints to enhance clarity.
   
   ```xml
   <!-- Consistent Naming Convention Example -->
   <link name="robot1_base_link"/>
   <joint name="robot1_arm_joint"/>
   ```

6. **Use of Xacro**: Utilize Xacro (XML Macros) to simplify URDF files by allowing macros and parameters, reducing redundancy.
   
   ```xml
   <!-- Example Xacro Macro for a Wheel -->
   <xacro:macro name="wheel" params="name">
       <link name="${name}_wheel">
           <!-- Wheel properties -->
       </link>
       <joint name="${name}_wheel_joint" type="revolute">
           <!-- Joint properties -->
       </joint>
   </xacro:macro>

   <!-- Using the Wheel Macro -->
   <xacro:wheel name="left"/>
   <xacro:wheel name="right"/>
   ```

---

## Conclusion

Creating a URDF file may seem daunting initially due to the many parameters and origins involved. However, with a step-by-step approach and adherence to best practices, you can efficiently define a robot’s structure and generate accurate TFs. This section covered the fundamentals of URDF, enabling you to start building and visualizing robotic models in ROS2. As you progress through this course, you will expand on this foundation to create more complex robots and simulations, leveraging the power of ROS2's TF and visualization tools to develop sophisticated and reliable autonomous systems.

---

## Additional Resources

- **URDF Tutorials**: [ROS2 URDF Tutorials](https://docs.ros.org/en/foxy/Tutorials.html#robot-description-formats)
- **Xacro Documentation**: [Xacro Overview](https://wiki.ros.org/xacro)
- **RViz Documentation**: [RViz User Guide](https://docs.ros.org/en/foxy/Tutorials/VisualizingRviz.html)

---

By following this guide and utilizing the provided examples, you will gain a solid understanding of how to create and manage URDF files, enabling you to effectively describe and visualize your robotic systems within ROS2.