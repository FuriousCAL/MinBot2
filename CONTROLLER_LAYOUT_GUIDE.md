# MinBot2 Controller Layout - Industry Standard (Following Elite FRC Teams)

## 🎮 Overview

This controller layout follows patterns established by championship FRC teams like 254 (The Cheesy Poofs), 971 (Spartan Robotics), 1678 (Citrus Circuits), and other top-tier teams. It provides both simple operations and advanced precision capabilities.

## 🚗 Drive Controls (Universal Standards)

| Control | Function | Details |
|---------|----------|---------|
| **Left Stick** | Translation (X/Y movement) | Standard across ALL elite teams |
| **Right Stick** | Rotation | Standard across ALL elite teams |
| **Left Trigger** | Precision Mode | Reduces speed to 30% (0.3x multiplier) |
| **Right Trigger** | Turbo Mode | Increases speed to 150% (1.5x multiplier) |
| **Left Bumper** | Field-Centric Toggle | Toggles between Robot and Field centric driving |

**Field-Centric**: Robot moves relative to field (forward = away from driver station)  
**Robot-Centric**: Robot moves relative to robot orientation (forward = robot's front)

## 🎯 Face Button Actions (Primary Controls)

| Button | Action | Purpose |
|--------|--------|---------|
| **A** | Cancel/Emergency Stop | Interrupts any active command + brief brake |
| **B** | Point Wheels | Points wheels toward left stick direction (hold) |
| **X** | Navigate to AprilTag #2 | Uses PathPlanner to position 2ft in front of Tag #2 |
| **Y** | Drive to Home Position | Returns to safe position (3,3,0°) |

## 🧭 D-Pad Navigation (Advanced Positioning)

**Following Elite Team Pattern**: D-pad for strategic positioning

| Direction | Action | AprilTag Target |
|-----------|---------|----------------|
| **Up** | Vision-Assisted Tag #1 | Blue alliance scoring position |
| **Left** | Vision-Assisted Tag #3 | Amp side positioning |  
| **Right** | Vision-Assisted Tag #4 | Source side positioning |
| **Down** | Emergency Home | Backup home command |

## 🔐 Safety Combinations (Elite Team Pattern)

| Combination | Action | Purpose |
|-------------|--------|---------|
| **RB + A** | Drive to Home | Alternative home command |
| **RB + X** | Precision Tag #2 | Enhanced vision-assisted navigation to Tag #2 |
| **RB (hold alone)** | Brake | Emergency stop (hold to maintain) |

## 🎯 Vision-Assisted vs Standard Navigation

### Standard Navigation (X button):
- Uses PathPlanner only
- Reliable even without vision
- Positions 2ft from AprilTag #2
- Good for general positioning

### Vision-Assisted Navigation (D-pad, RB+X):
- **Phase 1**: PathPlanner coarse navigation (gets within ~1.5m)
- **Phase 2**: Vision precision control (positions to 0.5m with 5cm accuracy)
- Automatic fallback if vision quality degrades
- Much more precise final positioning

## 📊 Real-Time Telemetry

Monitor these values on SmartDashboard during operation:

### Drive Status:
- `Drive Mode`: Robot/Field centric status
- `Robot Pose`: Current position on field
- `Vision/Tags Used`: Number of AprilTags being tracked

### Vision-Assisted Commands:
- `VisionAssisted/Phase`: Current phase (PATHPLANNER/VISION/FINISHED)
- `VisionAssisted/Status`: Current status updates
- `VisionAssisted/Target Tag`: Which AprilTag is being navigated to
- `VisionAssisted/Distance to Target`: Real-time distance measurement
- `VisionAssisted/Good Vision`: Vision quality indicator

## 🏆 Why This Layout (Elite Team Philosophy)

### 1. **Consistency Across Drivers**
- Any FRC driver familiar with top teams can immediately operate your robot
- Muscle memory transfers between robots using this standard

### 2. **Safety First**
- Emergency stop (A) easily accessible
- Brake (RB hold) prevents accidental activation
- Home commands provide safe fallback positions

### 3. **Graduated Complexity**
- Simple operations on face buttons
- Advanced operations on D-pad
- Precision variants on combinations

### 4. **Operational Efficiency**
- Most-used functions (drive, cancel, home) on primary buttons
- Strategic positioning (AprilTag navigation) on D-pad
- Emergency functions always accessible

## 🧪 Testing Your New Layout

### Basic Test:
1. **Drive Test**: Left/Right sticks for movement, triggers for speed
2. **Mode Toggle**: LB to switch between Robot/Field centric
3. **Emergency**: A button to cancel any active command

### Vision Test:
1. **Standard Navigation**: X button to navigate to Tag #2
2. **Vision-Assisted**: D-pad Up to navigate to Tag #1 with precision
3. **Precision Mode**: RB+X for enhanced Tag #2 navigation

### Safety Test:
1. **Emergency Stop**: A button during any movement
2. **Brake**: Hold RB (without other buttons) to brake
3. **Home**: Y button or RB+A to return to safe position

## 📈 Advanced Features

### Speed Scaling:
- **Normal**: 100% speed
- **Precision**: 30% speed (LT held)
- **Turbo**: 150% speed (RT held)
- **Combined**: Can combine precision + turbo for 45% speed

### Vision Quality Monitoring:
Vision-assisted commands automatically monitor:
- AprilTag visibility and clarity
- Distance to target for optimal vision
- Automatic fallback to PathPlanner if vision degrades

## 🔧 Customization Notes

All timing and distance values can be adjusted in the command files:

**VisionAssistedAprilTagCommand.java**:
- `TARGET_DISTANCE_METERS = 0.5`: Final distance from tag
- `VISION_SWITCH_DISTANCE = 1.5`: When to switch to vision control
- `MAX_VISION_AMBIGUITY = 0.3`: Vision quality threshold

**PathPlannerUtils.java**:
- Adjust speed and acceleration constraints for all PathPlanner movements

## 🎯 Competition Ready

This layout provides everything needed for FRC competition:
- ✅ Reliable manual driving with speed control
- ✅ Precision AprilTag navigation for scoring
- ✅ Emergency stops and safety measures  
- ✅ Strategic positioning for all game elements
- ✅ Consistent with championship team patterns

Your drivers will have confidence operating with familiar, proven patterns while gaining advanced vision capabilities that provide competitive advantages in precision positioning.
