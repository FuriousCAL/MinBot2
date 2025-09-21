# MinBot2 - Robot Capabilities and Controls

## 🤖 Robot Overview

MinBot2 is a sophisticated FRC 2025 robot featuring a swerve drivetrain with integrated PhotonVision for AprilTag navigation and autonomous capabilities.

---

## 🚀 Core Capabilities

### **Swerve Drivetrain**
- **4-wheel swerve drive** with CTRE Phoenix 6 motors
- **Field-centric and robot-centric** driving modes
- **Precision control** with configurable speed scaling
- **Smooth movement** in any direction with independent wheel control
- **Simulation-ready** with realistic physics modeling

### **Vision System Integration**
- **Real-time PhotonVision** camera integration (`ArduCam1`)
- **AprilTag detection** and pose estimation
- **Multi-tag pose estimation** for enhanced accuracy
- **Vision-assisted navigation** to specific field positions
- **Live camera monitoring** via PhotonVision dashboard

### **Autonomous Navigation**
- **PathPlanner integration** for dynamic pathfinding
- **AprilTag-based navigation** to field positions
- **Alliance-aware targeting** (automatically selects correct tags)
- **Obstacle avoidance** through intelligent path planning
- **Vision-assisted pose correction** during navigation

---

## 🎮 Control Layout

### **Primary Driving Controls**

| Control | Function | Description |
|---------|----------|-------------|
| **Left Stick** | Drive Forward/Back | Forward/backward movement |
| **Left Stick** | Strafe Left/Right | Side-to-side movement |
| **Right Stick** | Rotate | Robot rotation (Left = CCW, Right = CW) |
| **Left Trigger** | Precision Mode | Reduces speed to 30% for fine control |
| **Right Trigger** | Turbo Mode | Increases speed to 150% for fast movement |

### **Core Controls (Professional FRC Layout)**

| Control | Function | Description |
|---------|----------|-------------|
| **Left Bumper** | Toggle Drive Mode | Switch between Field-centric and Robot-centric |
| **Right Bumper** | Brake Mode | Emergency stop (hold to brake) |
| **A Button** | Cancel/Manual | Cancel all commands, return to manual control |
| **B Button** | Point Wheels | Align wheels toward left stick direction |
| **Y Button** | Home Position | Return to safe home position (3, 3) |

### **Vision-Assisted Navigation (Button Combinations)**

| Control | Function | Description |
|---------|----------|-------------|
| **X + Y** | Drive to AprilTag 2 | Navigate to Speaker AprilTag (main scoring) |
| **X + B** | Drive to AprilTag 1 | Navigate to AprilTag 1 (Blue alliance scoring) |
| **Y + B** | Drive to AprilTag 3 | Navigate to AprilTag 3 (Amp side) |
| **X + A** | Drive to AprilTag 4 | Navigate to AprilTag 4 (Source side) |

### **Safety and Testing Controls**

| Control | Function | Description |
|---------|----------|-------------|
| **Start Button** | Vision Test | Simple drive to AprilTag 2 using vision data |
| **Back Button** | Emergency Home | Backup home command for safety |
| **D-Pad Up** | Quick Home | Alternative home return |
| **D-Pad Down** | Vision Test | Alternative vision testing command |

---

## 📊 Vision System Capabilities

### **Real-Time Detection**
- **AprilTag recognition** with 2024 FRC field layout
- **Multi-tag pose estimation** for enhanced accuracy
- **Live camera feed** via PhotonVision dashboard
- **Detection overlays** showing identified targets
- **Pose estimation** with confidence scoring

### **Vision-Assisted Navigation**
- **Real camera data** driving simulation robot
- **Dynamic pose correction** during movement
- **Quality validation** of vision measurements
- **Graceful degradation** when vision is unavailable

### **Monitoring and Diagnostics**
- **Connection status** monitoring
- **Performance metrics** (latency, frame rate)
- **Target detection** statistics
- **Pose estimation** confidence levels

---

## 🎯 Autonomous Capabilities

### **Path Planning**
- **Dynamic pathfinding** to any field position
- **Obstacle avoidance** using PathPlanner algorithms
- **Speed and acceleration** constraints for safety
- **Smooth trajectory** generation and execution

### **AprilTag Navigation**
- **Alliance-aware targeting** (red vs blue alliance)
- **Multiple target types** (Speaker, Amp, Source, Stage)
- **Approach positioning** with configurable distances
- **Vision feedback** for precise positioning

### **Home Position System**
- **Safe home position** at (3, 3) on field
- **Automatic return** capability
- **Path-based navigation** to home
- **Emergency home** via multiple button combinations

---

## 🔧 Technical Specifications

### **Hardware**
- **Swerve Modules**: 4x CTRE Phoenix 6 TalonFX motors
- **Encoders**: CANcoder for precise wheel positioning
- **IMU**: Pigeon 2 for heading reference
- **Vision**: ArduCam1 with PhotonVision processing
- **Controller**: Xbox controller for teleop

### **Software**
- **WPILib 2025** with command-based architecture
- **PathPlanner 2025.2.7** for autonomous navigation
- **PhotonVision 2025.3.1** for vision processing
- **CTRE Phoenix 6** for motor control
- **Simulation support** for development and testing

### **Performance**
- **Maximum Speed**: 4.5 m/s (4.73 m/s theoretical)
- **Maximum Angular Rate**: 2π rad/s (360°/s)
- **Odometry Update**: 250 Hz
- **Vision Processing**: Real-time with <50ms latency
- **Path Planning**: Dynamic generation with obstacle avoidance

---

## 🚦 Operating Modes

### **TeleOperated Mode**
- **Manual control** via Xbox controller
- **Vision-assisted navigation** available
- **Real-time pose estimation** and correction
- **Safety features** and emergency stops

### **Autonomous Mode**
- **Pre-programmed routines** for competition
- **Vision-based positioning** for accuracy
- **Path planning** for complex maneuvers
- **Alliance-aware** target selection

### **Test Mode**
- **System validation** and calibration
- **Vision system testing** with real camera data
- **Path planning verification** in simulation
- **Performance monitoring** and diagnostics

---

## 📱 Monitoring and Telemetry

### **SmartDashboard Integration**
- **Real-time robot pose** display
- **Vision system status** and diagnostics
- **Target detection** information
- **Performance metrics** and health monitoring
- **Field2d widget** showing robot position

### **PhotonVision Dashboard**
- **Live camera feed** with AprilTag overlays
- **Detection statistics** and confidence levels
- **Pose estimation** visualization
- **System configuration** and calibration

### **Console Output**
- **Command execution** status and progress
- **Navigation updates** with position and error data
- **Vision system** health and performance
- **Error reporting** and troubleshooting information

---

## 🎮 Quick Start Guide

### **Basic Driving**
1. **Enable robot** in Driver Station
2. **Use left stick** for movement, **right stick** for rotation
3. **Hold left trigger** for precision mode
4. **Hold right trigger** for turbo mode

### **Vision Navigation**
1. **Ensure AprilTag is visible** to camera
2. **Press Start** for simple vision test
3. **Press X** for PathPlanner navigation to AprilTag 2
4. **Use D-pad** for other AprilTag targets

### **Emergency Procedures**
1. **Press A** to cancel any active commands
2. **Hold Right Bumper** for emergency brake
3. **Press Y** or **D-pad Down** to return home
4. **Press RB + A** for immediate home return

---

## 🔍 Troubleshooting

### **Vision Issues**
- Check `Vision/Connected` status in SmartDashboard
- Verify AprilTag visibility and lighting
- Monitor `Vision/Target Count` and `Vision/Best Target ID`
- Use PhotonVision dashboard for detailed diagnostics

### **Navigation Issues**
- Ensure robot is enabled and vision system connected
- Check target AprilTag is in field layout
- Verify PathPlanner configuration and constraints
- Monitor console output for error messages

### **Control Issues**
- Verify Xbox controller connection and mapping
- Check button combinations for safety features
- Ensure proper mode selection in Driver Station
- Monitor SmartDashboard for system status

---

**MinBot2 - Professional FRC Robot with Advanced Vision Integration** 🤖📷
