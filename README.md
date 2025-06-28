# 🤖 Spot ROS2 Driver - License-Free Version

<p align="center">
  <img src="spot.png" width="350">
  <h1 align="center">Spot ROS 2 - Modified Driver</h1>
</p>

A modified version of the Boston Dynamics Spot ROS2 driver that **removes the streaming license requirement** for basic arm and body control functionality.

## 🎯 Key Features

- ✅ **No Streaming License Required**: Works with basic Spot API (2Hz)
- ✅ **Arm Control**: Full 6-DOF arm movement via high-level API
- ✅ **ROS2 Control Integration**: Compatible with ros2_control ecosystem
- ✅ **Complete State Feedback**: Joint states, IMU, contact sensors
- ✅ **Docker Ready**: Complete containerized environment
- ✅ **Easy Setup**: Clone, build, and run

## 🚀 Quick Start

### Option 1: Docker (Recommended)

1. **Clone and Build**:
```bash
git clone --recursive https://github.com/murilo-vinicius04/spot-ros-no-license.git spot_ws
cd spot_ws
docker-compose up --build
```

2. **Enter Container**:
```bash
docker exec -it spot_ws_spot-ros2_1 bash
cd /workspaces/spot_ws
source install/setup.bash
```

3. **Connect to Real Robot**:
```bash
# Set environment variables
export BOSDYN_CLIENT_USERNAME=your_username
export BOSDYN_CLIENT_PASSWORD=your_password
export SPOT_IP=YOUR_ROBOT_IP

# Launch driver
ros2 launch spot_driver spot_driver.launch.py \
    launch_rviz:=true \
    robot_description_package:=spot_description \
    arm:=true
```

4. **Enable Control** (in another terminal):
```bash
# Stand up the robot
ros2 service call /stand std_srvs/srv/Trigger

# Launch ros2_control
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=robot
```

### Option 2: Native Installation

1. **Install ROS2 Humble** (Ubuntu 22.04)

2. **Clone and Build**:
```bash
mkdir -p ~/spot_ws && cd ~/spot_ws
git clone --recursive https://github.com/murilo-vinicius04/spot-ros-no-license.git .
chmod +x install_spot_ros2.sh
./install_spot_ros2.sh
colcon build --symlink-install
source install/setup.bash
```

## 🎮 Robot Control

### Basic Commands

```bash
# Robot commands
ros2 service call /stand std_srvs/srv/Trigger
ros2 service call /sit std_srvs/srv/Trigger

# Arm commands
ros2 service call /arm_stow std_srvs/srv/Trigger
ros2 service call /arm_unstow std_srvs/srv/Trigger
```

### Advanced Joint Control

```bash
# Full robot position command (19 joints: 12 legs + 7 arm)
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
'{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -0.3, 0.8, 0.0, 0.3, 0.0, 0.0]}'
```

### Environment Setup

```bash
# Set environment variables for your robot
export BOSDYN_CLIENT_USERNAME="your_username"      # Default: admin
export BOSDYN_CLIENT_PASSWORD="your_password"      # Default: <your_robot_password>
export SPOT_IP="YOUR_ROBOT_IP"                     # Example: 192.168.80.3
export SPOT_PORT="443"                             # Optional, defaults to 443
```

**Note**: Replace the placeholder values with your actual robot credentials and IP address.

## 🛠️ System Architecture

### Core Packages

| Package | Description |
|---------|-------------|
| `spot_driver` | Main driver for Spot communication |
| `spot_hardware_interface` | **Modified** hardware interface (license-free) |
| `spot_ros2_control` | ROS2 control integration |
| `spot_controllers` | Custom controllers |
| `spot_msgs` | Custom message definitions |
| `spot_description` | Robot URDF and visualization |

### Modified Components

The key modification is in `spot_hardware_interface.cpp`:

- **Replaced streaming API** (requires license) with **basic API** (2Hz)
- **High-level arm commands** using protobuf messages
- **State polling** instead of streaming for joint feedback
- **Rate limiting** to comply with basic API restrictions

## ⚙️ Configuration

### Robot Connection

Set environment variables:

```bash
# Configure robot connection
export BOSDYN_CLIENT_USERNAME="your_username"      # Default: admin  
export BOSDYN_CLIENT_PASSWORD="your_password"      # Default: <your_robot_password>
export SPOT_IP="YOUR_ROBOT_IP"                     # Example: 192.168.80.3
export SPOT_PORT="443"                             # Optional, defaults to 443
```

### Joint Ordering

```
Legs [0-11]:   FL_hip_x, FL_hip_y, FL_knee,    # Front Left
               FR_hip_x, FR_hip_y, FR_knee,    # Front Right
               RL_hip_x, RL_hip_y, RL_knee,    # Rear Left
               RR_hip_x, RR_hip_y, RR_knee     # Rear Right

Arm [12-18]:   sh0, sh1, el0, el1, wr0, wr1, f1x
```

## 🔄 Simulation Mode

For development without a real robot:

```bash
# Mock mode with full robot simulation
ros2 launch spot_ros2_control spot_ros2_control.launch.py \
    hardware_interface:=mock \
    mock_arm:=true \
    launch_rviz:=true
```

## 📊 API Differences

| Feature | Streaming API (Licensed) | Basic API (License-Free) |
|---------|-------------------------|--------------------------|
| **Frequency** | 50Hz | 2Hz |
| **Latency** | ~20ms | ~500ms |
| **Movement** | Smooth interpolation | Step-wise |
| **Cost** | Requires license | Free |
| **Use Case** | Production | Development/Education |

## 🚨 Troubleshooting

### Connection Issues

```bash
# Check robot connectivity
ping $SPOT_IP

# Verify robot status
curl -k -u $BOSDYN_CLIENT_USERNAME:$BOSDYN_CLIENT_PASSWORD https://$SPOT_IP:443/api/v1/status
```

### Common Problems

1. **Robot not responding**: Check network connection and credentials
2. **Movements are jerky**: Expected with 2Hz API, normal behavior
3. **Commands rejected**: Some extreme positions filtered by safety systems
4. **Container issues**: Restart with `docker-compose restart`

### Diagnostic Commands

```bash
# Check robot connectivity
ping $SPOT_IP

# Verify robot status via API
curl -k -u $BOSDYN_CLIENT_USERNAME:$BOSDYN_CLIENT_PASSWORD https://$SPOT_IP:443/api/v1/status
```

## 📋 Requirements

- **OS**: Ubuntu 22.04
- **ROS**: ROS2 Humble
- **Python**: 3.10+
- **Network**: Connection to Spot robot
- **Hardware**: Any x86_64 or ARM64 system

## 🔧 Development

### Building Changes

```bash
# After modifying code
cd ~/spot_ws
colcon build --packages-select spot_hardware_interface
source install/setup.bash
```

### Adding New Features

The main modification points:
- `src/spot_ros2/spot_hardware_interface/src/spot_hardware_interface.cpp`
- Functions: `send_arm_command_basic()`, `basic_state_loop()`

## 📄 License

This project maintains the original licensing:
- **BSD3**: Parts derived from Clearpath Robotics ROS1 driver
- **MIT**: ROS2-specific developments

## 🤝 Contributing

1. Fork this repository
2. Create feature branch
3. Test with both simulation and real robot
4. Submit pull request with detailed description

## ⚠️ Important Notes

- **2Hz Limitation**: This version operates at 2Hz maximum due to basic API constraints
- **Production Use**: For production applications requiring smooth movement, consider the streaming license
- **Safety**: Always test in simulation first, use e-stop when available
- **Network**: Stable network connection essential for real robot operation

## 📞 Support

For issues with this modified driver:
1. Check troubleshooting section
2. Verify robot connectivity with diagnostic scripts
3. Test in simulation mode first
4. Review logs with `ros2 node list` and `ros2 topic list`

---

**🎯 Summary**: Clone → Build → Connect → Control
**🛡️ Safety First**: Always test in simulation before real robot use
**💡 Development**: Perfect for learning and prototyping without license costs 