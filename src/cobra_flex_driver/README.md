# Cobra Flex ROS2 Driver

ROS2 driver package for the Waveshare Cobra Flex chassis with JSON serial communication protocol.

## Overview

This package provides ROS2 integration for the Waveshare Cobra Flex 4WD chassis platform. It communicates with the ESP32-S3 based lower computer using Waveshare's JSON protocol over serial.

**Features:**
- Standard ROS2 interfaces (`/cmd_vel`, `/odom`, `/imu`, `/joint_states`)
- TF broadcasting (`odom` → `base_link`)
- Configurable robot parameters
- Safety watchdog timer
- Compatible with ROS2 Humble (and newer)

## Hardware Requirements

- Waveshare Cobra Flex chassis
- Upper computer: Raspberry Pi 4/5 or NVIDIA Jetson Orin Nano
- USB cable for serial communication with ESP32

## Dependencies

```bash
sudo apt install ros-humble-geometry-msgs \
                 ros-humble-nav-msgs \
                 ros-humble-sensor-msgs \
                 ros-humble-tf2-ros \
                 python3-serial
```

## Installation

### From Source

1. Create a workspace and clone the package:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Extract the cobra_flex_driver package here
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select cobra_flex_driver
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch cobra_flex_driver cobra_flex.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch cobra_flex_driver cobra_flex.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200 \
    publish_rate:=50.0
```

### Run Node Directly

```bash
ros2 run cobra_flex_driver cobra_flex_node
```

### Using Parameter File

```bash
ros2 launch cobra_flex_driver cobra_flex.launch.py \
    params_file:=src/cobra_flex_driver/config/cobra_flex_params.yaml
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port for ESP32 |
| `baud_rate` | `115200` | Serial communication baud rate |
| `odom_frame` | `odom` | Odometry frame ID |
| `base_frame` | `base_link` | Robot base frame ID |
| `publish_tf` | `true` | Publish TF transforms |
| `cmd_vel_timeout` | `1.0` | Timeout for velocity commands (seconds) |
| `publish_rate` | `20.0` | Publishing frequency (Hz) |
| `use_sim_time` | `false` | Use simulation time |

## ROS2 Topics

### Subscribed Topics

- `/cmd_vel` (`geometry_msgs/Twist`) - Velocity commands for the robot

### Published Topics

- `/odom` (`nav_msgs/Odometry`) - Wheel odometry
- `/imu/data` (`sensor_msgs/Imu`) - IMU sensor data
- `/joint_states` (`sensor_msgs/JointState`) - Wheel joint positions and velocities
- `/tf` - Transform from `odom` to `base_link`

## Testing

### 1. Check Topics

```bash
# List active topics
ros2 topic list

# Echo odometry
ros2 topic echo /odom

# Echo IMU data
ros2 topic echo /imu/data

# Monitor command velocity
ros2 topic echo /cmd_vel
```

### 2. Test with Keyboard Teleop

```bash
# Install teleop package (if not already installed)
sudo apt install ros-humble-teleop-twist-keyboard

# Terminal 1: Launch driver
ros2 launch cobra_flex_driver cobra_flex.launch.py

# Terminal 2: Launch keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Visualise in RViz2

```bash
# Launch RViz2
ros2 run rviz2 rviz2

# Add displays:
# - TF
# - Odometry (/odom)
# - IMU (/imu/data)
```

### 4. Check TF Tree

```bash
# Install tf2_tools if needed
sudo apt install ros-humble-tf2-tools

# View TF tree
ros2 run tf2_tools view_frames

# Check transform
ros2 run tf2_ros tf2_echo odom base_link
```

## JSON Protocol

### Commands to ESP32

**Velocity Command:**
```json
{
  "T": 1,
  "L": 50,    // Linear command (-100 to 100)
  "R": 0      // Angular command (-100 to 100)
}
```

**Configuration Command:**
```json
{
  "T": 900,
  "main": 4,   // Robot type
  "module": 0  // Module type
}
```

### Expected Responses from ESP32

```json
{
  "encoder": [fl, fr, rl, rr],  // Encoder counts
  "velocity": [fl, fr, rl, rr],  // Wheel velocities (rad/s)
  "imu": {
    "gyro": [x, y, z],           // Angular velocity (rad/s)
    "accel": [x, y, z],          // Linear acceleration (m/s²)
    "orientation": [x, y, z, w]  // Quaternion
  },
  "battery": 12.0                // Voltage
}
```

## Troubleshooting

### Serial Port Permission Denied

```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# Log out and back in, or:
sudo chmod 666 /dev/ttyUSB0
```

### Find Correct Serial Port

```bash
# List USB devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Or use dmesg
dmesg | grep tty
```

### Test Serial Connection

```bash
# Install screen
sudo apt install screen

# Connect to serial port
screen /dev/ttyUSB0 115200

# Exit: Ctrl+A then K
```

### No Data Published

1. Check serial connection: `ls /dev/ttyUSB*`
2. Verify baud rate matches ESP32 firmware
3. Check ESP32 firmware is flashed
4. Monitor ROS2 logs: `ros2 node info /cobra_flex_driver`

### Robot Not Moving

1. Check battery voltage
2. Verify motor connections
3. Test with manual JSON commands
4. Check cmd_vel topic: `ros2 topic echo /cmd_vel`

## Integration with Navigation Stack

This driver is compatible with Nav2. Example configuration:

```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      robot_base_frame: base_link
      global_frame: odom
```

Launch with Nav2:
```bash
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=nav2_params.yaml
```

## Development

### Package Structure

```
cobra_flex_driver/
├── cobra_flex_driver/
│   ├── __init__.py
│   └── cobra_flex_node.py
├── launch/
│   └── cobra_flex.launch.py
├── config/
│   └── cobra_flex_params.yaml
├── resource/
│   └── cobra_flex_driver
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

### Building for Development

```bash
cd ~/ros2_ws
colcon build --packages-select cobra_flex_driver --symlink-install
```

The `--symlink-install` flag allows you to edit Python files without rebuilding.

## Known Issues

- **JSON Protocol:** The exact JSON format may vary depending on ESP32 firmware version. Adjust `parse_json_response()` as needed.
- **Encoder Resolution:** Default `counts_per_rev = 1440` may need adjustment for your specific motors.
- **Coordinate Frames:** Ensure IMU orientation matches robot coordinate conventions.

## TODO / Future Enhancements

- [ ] Add ros2_control hardware interface
- [ ] Battery state monitoring and publishing
- [ ] Servo control for camera pan-tilt
- [ ] LED control interface
- [ ] Diagnostics publishing
- [ ] Dynamic reconfigure support
- [ ] Unit tests
- [ ] Hardware-in-the-loop testing

## Contributing

Contributions are welcome! Please:
1. Test with actual Cobra Flex hardware
2. Validate JSON protocol with your ESP32 firmware version
3. Submit issues and pull requests

## License

GPL-3.0 - Following Waveshare's open-source licensing

## References

- [Waveshare Cobra Flex Wiki](https://www.waveshare.com/wiki/Cobra_Flex)
- [Waveshare Cobra Flex Product Page](https://www.waveshare.com/cobra-flex.htm)
- [Waveshare UGV Base ROS GitHub](https://github.com/waveshareteam/ugv_base_ros)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## Support

For issues specific to this package, please open an issue on GitHub.

For Waveshare hardware support, use their official support channels:
- [Waveshare Support](https://support.waveshare.com/)

## Authors

- L-CAS, University of Lincoln

## Acknowledgements

Based on Waveshare's open-source UGV architecture and JSON protocol.
