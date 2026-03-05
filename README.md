# LC3 ROS2 Hardware Interface

ROS2 hardware interface for LC3 lifting column using Modbus TCP communication.

## Overview

This package provides a ros2_control hardware interface for controlling LC3 linear actuators/lifting columns via Modbus TCP protocol.

## Features

- Full ros2_control SystemInterface implementation
- Modbus TCP communication
- Position and velocity feedback
- Heartbeat mechanism for safe operation
- Send-only-on-change optimization
- Automatic retraction on shutdown
- Standalone URDF and launch file for independent use

## Dependencies

- ROS2 Humble
- libmodbus (install: `sudo apt install libmodbus-dev`)
- ros2_control
- ros2_controllers
- hardware_interface
- pluginlib
- controller_manager
- robot_state_publisher
- xacro

## Configuration

The hardware interface connects to the LC3 column at:
- IP: 192.168.1.10
- Port: 502
- Slave ID: 1

These can be modified in `include/lc3_hw_interface/lc3_hardware_interface.hpp` if needed.

## Build

```bash
cd ~/ros2_yeray
colcon build --packages-select lc3_hw_interface
source install/setup.bash
```

## Usage

### Standalone Mode

Launch the LC3 column controller independently:

```bash
ros2 launch lc3_hw_interface lc3_column.launch.py
```

**With mock hardware (for testing without real hardware):**
```bash
ros2 launch lc3_hw_interface lc3_column.launch.py use_mock_hardware:=true
```

### Control the Column

Send position commands (values in meters, range: 0.0 to 0.9):

```bash
# Extend to 10 cm
ros2 topic pub --once /column_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1]"

# Retract to 0
ros2 topic pub --once /column_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]"

# Extend to maximum (90 cm)
ros2 topic pub --once /column_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.9]"
```

### Monitor Joint States

```bash
ros2 topic echo /joint_states
```

### Integration with Your Robot

Add to your robot's URDF/xacro:

```xml
<ros2_control name="lc3_column_controller" type="system">
  <hardware>
    <plugin>lc3_hw_interface/LC3HardwareInterface</plugin>
    <param name="column_name">column_joint</param>
  </hardware>
  <joint name="column_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## Files

- `urdf/lc3_column.urdf.xacro` - Robot description with LC3 column
- `config/lc3_controllers.yaml` - Controller configuration
- `launch/lc3_column.launch.py` - Launch file to start everything
- `src/lc3_hardware_interface.cpp` - Hardware interface implementation
- `include/lc3_hw_interface/lc3_hardware_interface.hpp` - Header file

## Safety

- The column automatically retracts to position 0 on shutdown (Ctrl+C)
- Position limits enforced: 0.0 m (retracted) to 0.9 m (extended)
- Heartbeat mechanism ensures continuous communication
- Send-only-on-change reduces unnecessary Modbus traffic

## License

Apache-2.0
