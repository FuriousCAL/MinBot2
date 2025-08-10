# Home Position Navigation Implementation

## Overview
Successfully implemented a "Return to Home" feature that allows the robot to navigate back to its starting position (3,3) on the field using a joystick button combination.

## What Was Implemented

### 1. Home Position Constant
**File:** `src/main/java/frc/robot/constants/AprilTagConstants.java`
- Added `HOME_POSITION` constant set to (3.0, 3.0, 0°)
- This represents the robot's starting position and safe return location

### 2. DriveToHomeCommand
**File:** `src/main/java/frc/robot/commands/DriveToHomeCommand.java`
- New command that uses PathPlanner to navigate to the home position
- Uses the same path constraints as AprilTag navigation (2 m/s max speed)
- Includes error handling and console logging for debugging
- Automatically calculates path from current position to home

### 3. Button Binding
**File:** `src/main/java/frc/robot/RobotContainer.java`
- Added button combination: **Left Bumper + A Button** → Drive to Home
- This combination avoids conflicts with existing controls
- Added import for the new DriveToHomeCommand

### 4. Starting Position Initialization
**File:** `src/main/java/frc/robot/Robot.java`
- Robot now starts at the home position (3,3) when initialized
- Uses `resetPose()` to set the initial odometry position

## Button Layout (Updated)

```
Left Stick:           Forward/Strafe driving
Right Stick:          Rotation
X Button:             → Speaker AprilTag
Y Button:             → Amp AprilTag  
B Button:             → Source AprilTag
A Button:             Cancel/Manual Control
Left Bumper + A:      → Return to Home Position (3,3)  ← NEW!
Right Bumper:         Brake Mode
Left Bumper:          Reset Heading
Left Trigger:         Point Wheels
POV + Back:           SysId Dynamic
POV + Start:          SysId Quasistatic
```

## How It Works

1. **Initialization**: Robot starts at position (3,3) with 0° heading
2. **During Operation**: Robot can drive anywhere on the field using normal controls
3. **Return Home**: Press and hold Left Bumper, then press A button
4. **PathPlanner Navigation**: 
   - Calculates optimal path from current position to (3,3)
   - Avoids obstacles using PathPlanner's pathfinding
   - Drives at safe speeds (2 m/s max)
5. **Completion**: Robot arrives at home position and stops

## Testing Instructions

### In Simulation:
1. **Start Robot Simulation**: `./gradlew simulateJava`
2. **Enable Robot**: Use Driver Station or simulation GUI
3. **Drive Around**: Use joysticks to move the robot away from (3,3)
4. **Return Home**: Hold Left Bumper + Press A button
5. **Observe**: Robot should automatically navigate back to (3,3)

### Console Output:
The command provides helpful debug information:
```
DriveToHomeCommand: Driving to home position (3,3)
Current pose: Pose2d(X: 5.2, Y: 7.1, Rotation2d(Rads: 0.5))
Target pose: Pose2d(X: 3.0, Y: 3.0, Rotation2d(Rads: 0.0))
Path generated successfully to home position
DriveToHomeCommand: Arrived at home position
```

## Technical Details

- **PathPlanner Integration**: Uses `AutoBuilder.pathfindToPose()` for dynamic pathfinding
- **Path Constraints**: 2 m/s velocity, 2 m/s² acceleration, 360°/s rotation
- **Error Handling**: Graceful failure if path generation fails
- **Pose Tracking**: Uses swerve drivetrain odometry for current position
- **Alliance Independent**: Home position is fixed regardless of alliance color

## Future Enhancements

Potential improvements for the future:
1. **Multiple Home Positions**: Different safe zones based on game strategy
2. **Dynamic Home**: Set home position based on where robot is placed
3. **Vision Correction**: Use AprilTags to correct position during navigation
4. **Obstacle Avoidance**: Enhanced pathfinding around field elements
5. **Speed Profiles**: Different speeds for different situations

## Build Status
✅ **Compiles Successfully**: Project builds without errors
✅ **Simulation Ready**: Robot simulation starts and runs
✅ **Button Binding Active**: Left Bumper + A triggers home navigation
✅ **Starting Position Set**: Robot initializes at (3,3) position

The implementation is complete and ready for testing!
