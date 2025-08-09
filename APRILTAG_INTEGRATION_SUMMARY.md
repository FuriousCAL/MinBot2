# AprilTag Navigation Integration - Phase 1 Complete

## What We've Implemented

### 1. AprilTag Constants (`src/main/java/frc/robot/constants/AprilTagConstants.java`)
- **Field Layout**: Uses standard FRC 2024 Crescendo field AprilTag layout
- **Tag IDs**: Defined constants for all field AprilTags (Speaker, Amp, Source, Stage)
- **Approach Configuration**: Configurable approach distances and angles for different tag types
- **Smart Positioning**: `getApproachPose()` method calculates optimal approach positions

### 2. DriveToAprilTag Command (`src/main/java/frc/robot/commands/DriveToAprilTagCommand.java`)
- **PathPlanner Integration**: Uses PathPlanner 2025.2.7 for on-the-fly path generation
- **Alliance Awareness**: Automatically selects correct tags based on alliance color
- **Flexible Targeting**: Can target specific tag IDs or tag types (Speaker, Amp, Source)
- **Error Handling**: Graceful failure handling if path generation fails
- **Path Constraints**: Configurable speed and acceleration limits for safe navigation

### 3. Updated Button Bindings (`src/main/java/frc/robot/RobotContainer.java`)
- **X Button**: Drive to Speaker AprilTag (alliance-specific)
- **Y Button**: Drive to Amp AprilTag (alliance-specific)  
- **B Button**: Drive to Source AprilTag (alliance-specific)
- **A Button**: Cancel navigation / return to manual control
- **Right Bumper**: Brake mode (moved from A button)
- **Left Trigger**: Point wheels toward heading (moved from B button)
- **Left Bumper**: Reset field-centric heading (unchanged)
- **SysId**: Moved to POV buttons to free up X/Y buttons

## Current Status

✅ **Build Success**: Project compiles without errors
✅ **Simulation Ready**: Robot simulation starts successfully
✅ **PathPlanner Integration**: Commands use PathPlanner for path generation
✅ **Alliance Awareness**: Automatically selects correct tags based on alliance
✅ **Joystick Controls**: All basic swerve controls working + new AprilTag navigation

## Testing Instructions

### In Simulation:
1. **Start Robot Simulation**: `./gradlew simulateJava`
2. **Enable Robot**: Use Driver Station or simulation GUI
3. **Test Basic Drive**: Use left stick (drive) and right stick (rotate)
4. **Test AprilTag Navigation**:
   - Press **X** to drive to Speaker
   - Press **Y** to drive to Amp  
   - Press **B** to drive to Source
   - Press **A** to cancel and return to manual control

### Button Layout Summary:
```
Left Stick:     Forward/Strafe driving
Right Stick:    Rotation
X Button:       → Speaker AprilTag
Y Button:       → Amp AprilTag
B Button:       → Source AprilTag  
A Button:       Cancel/Manual Control
Right Bumper:   Brake Mode
Left Bumper:    Reset Heading
Left Trigger:   Point Wheels
POV + Back:     SysId Dynamic
POV + Start:    SysId Quasistatic
```

## Next Steps (Phase 2)

### PhotonVision Integration:
1. Create `VisionSubsystem` for AprilTag detection
2. Implement vision-based pose estimation
3. Integrate vision measurements into swerve odometry
4. Add vision-assisted navigation commands
5. Implement dynamic path correction based on vision feedback

### Files to Create:
- `src/main/java/frc/robot/constants/VisionConstants.java`
- `src/main/java/frc/robot/subsystems/VisionSubsystem.java`
- `src/main/java/frc/robot/commands/ApproachAprilTagCommand.java` (vision-assisted)

## Technical Notes

- **PathPlanner API**: Using 2025.2.7 with simplified configuration due to API changes
- **CTRE Phoenix 6**: Full compatibility maintained with existing swerve drivetrain
- **Error Handling**: All PathPlanner operations wrapped in try-catch blocks
- **Performance**: Path generation happens on-demand when buttons are pressed
- **Alliance Color**: Automatically detected from DriverStation for tag selection

## Known Limitations

1. **PathPlanner Configuration**: Simplified due to 2025.2.7 API changes - will be enhanced in Phase 2
2. **No Vision Feedback**: Currently uses odometry only - PhotonVision integration in Phase 2
3. **Static Approach**: Fixed approach distances - will be made dynamic with vision feedback
4. **No Obstacle Avoidance**: Basic pathfinding only - advanced features in Phase 3

The foundation is now in place for AprilTag navigation. The robot can drive to any AprilTag on the field using PathPlanner, with proper alliance awareness and joystick integration.
