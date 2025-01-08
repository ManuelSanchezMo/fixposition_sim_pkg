
# FixPosition Simulator

## Overview

The **FixPosition Simulator** is a ROS 2-based tool designed to emulate real-world positioning and odometry data for testing and development purposes. It publishes simulated data on various topics, enabling developers to test navigation, localization, and other robotics applications without the need for physical hardware.

## Simulator Setup

To integrate the **FixPosition Simulator** with your robot, add the following `<xacro:fixposition_full_setup>` tag to your robot's URDF or Xacro file:

```xml
<xacro:fixposition_full_setup 
    parent="Bridge" 
    xyz="-0.0028148 -0.11697 0.21171" 
    rpy="1.5708 0 1.5708" 
    poi_frame_xyz="10.0 0.05 0.3" 
    poi_frame_rpy="0 0 0" 
    fusion_rate="30.0"/>
```

**Parameters Explained:**

- **parent** (`string`):  
  The name of the parent link in your robot's URDF to which the simulator's sensor is attached.  
  *Example*: `parent="Bridge"`

- **xyz** (`double[3]`):  
  The position of the sensor relative to the parent link, specified as X, Y, and Z coordinates in meters.  
  *Example*: `xyz="-0.0028148 -0.11697 0.21171"`

- **rpy** (`double[3]`):  
  The orientation of the sensor in Roll, Pitch, and Yaw angles (in radians) relative to the parent link.  
  *Example*: `rpy="1.5708 0 1.5708"`

- **poi_frame_xyz** (`double[3]`):  
  The position of the POI (Point of Interest) frame relative to the sensor, specified as X, Y, and Z coordinates in meters.  
  *Example*: `poi_frame_xyz="10.0 0.05 0.3"`

- **poi_frame_rpy** (`double[3]`):  
  The orientation of the POI frame in Roll, Pitch, and Yaw angles (in radians) relative to the sensor.  
  *Example*: `poi_frame_rpy="0 0 0"`

- **fusion_rate** (`double`):  
  The rate at which the fusion sensor operates, measured in Hertz (Hz).  
  *Example*: `fusion_rate="30.0"`

**Example Integration:**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Include FixPosition Simulator Macros -->
  <xacro:include filename="$(find fixposition_simulator)/urdf/fixposition_macros.xacro"/>

  <!-- Other robot components -->

  <!-- FixPosition Simulator Setup -->
  <xacro:fixposition_full_setup 
      parent="base_link" 
      xyz="0.1 0.0 0.2" 
      rpy="0 0 0" 
      poi_frame_xyz="5.0 0.0 0.0" 
      poi_frame_rpy="0 0 0" 
      fusion_rate="30.0"/>
  
</robot>
```

**Notes:**

- Ensure that the `fixposition_macros.xacro` file containing the `fixposition_full_setup` macro is correctly referenced and available in your project.
- Adjust the parameter values (`parent`, `xyz`, `rpy`, `poi_frame_xyz`, `poi_frame_rpy`, `fusion_rate`) to match your robot's specific configuration and requirements.

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/fixposition_simulator.git
```

### 2. Install Dependencies

Ensure all necessary dependencies are installed:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package

Use `colcon` to build the workspace:

```bash
colcon build --packages-select fixposition_simulator
```

### 4. Source the Workspace

After building, source the workspace to overlay the new package:

```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Running the Simulator

Launch the FixPosition Simulator node using `ros2 run`:

```bash
ros2 run fixposition_simulator odom_simulator_node
```

### Configuring Publish Rates

Set custom publish rates for different data streams via parameters:

```bash
ros2 run fixposition_simulator odom_simulator_node --ros-args -p enu_publish_rate:=10 -p ecef_publish_rate:=5
```

## Topics

The FixPosition Simulator publishes data on the following ROS 2 topics:

| Topic                        | Message Type               | Description                          |
| ---------------------------- | -------------------------- | ------------------------------------ |
| `/fixposition/fpa/odomenu`   | `fixposition_msgs/ODOMENU` | Simulated odometry in ENU coordinates|
| `/fixposition/odometry_ecef`| `nav_msgs/Odometry`        | Simulated odometry in ECEF coordinates|

*Adjust the topics based on your implementation.*

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For questions or issues, please open an [issue](https://github.com/yourusername/fixposition_simulator/issues) on GitHub or contact the maintainer at [youremail@example.com](mailto:youremail@example.com).
