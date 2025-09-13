# MinBot2 - FRC Team Robot Code

Welcome to the MinBot2 robot code repository. This project follows industry best practices adopted from top FRC teams to ensure maintainable, reliable, and professional code.

## Table of Contents

- [Getting Started](#getting-started)
- [Project Structure](#project-structure)
- [Development Workflow](#development-workflow)
- [Code Standards](#code-standards)
- [Building and Deploying](#building-and-deploying)
- [Testing](#testing)
- [Contributing](#contributing)

## Getting Started

### Prerequisites

- **WPILib 2025**: Install the latest WPILib suite
- **Java 17**: Included with WPILib installation
- **Git**: For version control
- **VS Code**: Recommended IDE with WPILib extensions

### Initial Setup

1. Clone the repository:
```bash
git clone https://github.com/FuriousCAL/MinBot2.git
cd MinBot2
```

2. Open in VS Code:
```bash
code .
```

3. Build the project:
```bash
./gradlew build
```

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java              # Main robot class
├── RobotContainer.java     # Robot subsystem and command container
├── Main.java              # Application entry point
├── Telemetry.java         # Data logging and telemetry
├── commands/              # Command implementations
├── subsystems/            # Robot subsystems
├── constants/             # Robot constants and configuration
├── utils/                 # Utility classes
└── generated/             # Auto-generated configuration files

src/main/deploy/           # Files deployed to robot
└── pathplanner/           # PathPlanner autonomous configurations

vendordeps/                # Third-party library dependencies
```

## Development Workflow

### Code Organization Standards

1. **Package Structure**: Follow the standard FRC package layout
2. **Naming Conventions**: Use descriptive, consistent naming
3. **File Organization**: Group related functionality together
4. **Documentation**: Document all public methods and complex logic

### Git Workflow

1. Create feature branches for new work:
```bash
git checkout -b feature/new-functionality
```

2. Make small, focused commits with clear messages:
```bash
git commit -m "Add swerve drive auto-alignment command"
```

3. Push and create pull requests for code review

## Code Standards

### Java Style Guidelines

- **Indentation**: 4 spaces (no tabs)
- **Line Length**: 120 characters maximum
- **Braces**: Opening brace on same line
- **Naming**:
  - Classes: `PascalCase`
  - Methods/Variables: `camelCase`
  - Constants: `UPPER_SNAKE_CASE`
  - Packages: `lowercase`

### Documentation Requirements

- All public methods must have Javadoc comments
- Complex algorithms should include inline comments
- README files in subsystem directories explaining functionality
- Constants should be well-documented with units and purposes

### Example Code Style:

```java
/**
 * Drives the robot to a specified AprilTag with offset positioning.
 * 
 * @param targetId The ID of the target AprilTag
 * @param xOffset Offset in meters from the tag (positive = away from tag)
 * @param yOffset Lateral offset in meters (positive = left of tag)
 * @return Command that completes when robot reaches target position
 */
public Command driveToAprilTagWithOffset(int targetId, double xOffset, double yOffset) {
    return new DriveToAprilTagOffsetCommand(
        drivetrain, 
        visionSubsystem, 
        targetId, 
        xOffset, 
        yOffset
    );
}
```

## Building and Deploying

### Build Commands

```bash
# Build the project
./gradlew build

# Deploy to robot
./gradlew deploy

# Simulate robot code
./gradlew simulateJava
```

### Robot Configuration

- **Team Number**: Configured in `.wpilib/wpilib_preferences.json`
- **Robot Type**: Swerve drive with vision processing
- **Control System**: Command-based architecture

## Testing

### Unit Testing

- Write unit tests for utility classes and command logic
- Use WPILib's testing framework for robot-specific testing
- Aim for good test coverage on critical functionality

### Integration Testing

- Test subsystem interactions in simulation
- Validate autonomous routines before competition
- Test edge cases and error conditions

### Running Tests

```bash
# Run all tests
./gradlew test

# Run specific test class
./gradlew test --tests CommandTest
```

## Contributing

### Code Review Process

1. All code must be reviewed before merging
2. Review criteria:
   - Functionality correctness
   - Code style adherence
   - Proper documentation
   - Test coverage
   - Performance considerations

### Pull Request Guidelines

- Keep changes focused and small
- Include descriptive commit messages
- Update documentation as needed
- Add tests for new functionality
- Ensure all existing tests pass

### Development Best Practices

1. **Start Small**: Implement and test simple functions first
2. **Incremental Changes**: Make small, verifiable improvements
3. **Test Early**: Test functionality as soon as possible
4. **Document Everything**: Keep documentation current
5. **Follow Standards**: Consistency across the codebase

## Key Features

- **Swerve Drive**: Advanced drivetrain with independent wheel steering
- **Vision Processing**: AprilTag detection and positioning
- **Autonomous Navigation**: PathPlanner integration for complex routes
- **Telemetry**: Comprehensive data logging for analysis
- **Modular Design**: Easy to extend and maintain

## Competition Preparation

- Test all autonomous routines thoroughly
- Verify all systems in various lighting conditions
- Practice driver controls and emergency stops
- Document any known issues or workarounds

## Support

For questions or issues:
1. Check existing documentation
2. Review similar implementations in the codebase
3. Ask team mentors or experienced developers
4. Create detailed issue reports for bugs

---

**Remember**: Professional, maintainable code wins competitions. Follow these standards to ensure our robot performs reliably when it matters most.
