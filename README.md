# Rebuilt2026 Robot Code

This repository contains the official software for the Rebuilt2026 FRC robot for team 5533, developed using the WPILib Command-Based framework. This code is engineered for robust performance in competition, emphasizing reliability, debuggability, and adherence to best practices in FRC software development.

## Project Overview

**Team**: Robotics 5533  
**Year**: 2026  
**Framework**: WPILib Command-Based Robot Framework  
**Language**: Java  
**Build System**: Gradle  
**Branch**: TestingV2

### Robot Capabilities
- **Swerve Drive**: CTRE Falcon 500-based swerve drivetrain with field-centric control
- **Shooter System**: Dual motor shooter with interpolated distance-based RPS adjustment
- **Intake Mechanism**: Motorized intake with flip position (in/out/erect)
- **Vision System**: Limelight integration for target tracking and autonomous alignment
- **Feeding System**: Dual feeder motors and washer motor for precise game piece management

## Getting Started

### Prerequisites
- WPILib 2026 or newer installed
- Java Development Kit (JDK) 21+
- VS Code with WPILib extension

### Setup Instructions
1. Clone this repository to your local machine
2. Open the project directory in VS Code
3. Run `./gradlew build` to compile the project
4. Deploy to the robot with `./gradlew deploy`

## Codebase Navigation

The robot's software is structured according to the WPILib Command-Based paradigm, promoting modularity and clear separation of concerns.

*   **Core Structure**: The primary Java source code resides in `src/main/java/frc/robot/`.
*   **Central Configuration**: Global constants, motor IDs, PID gains, and physical parameters are defined in [`Constants.java`](src/main/java/frc/robot/Constants.java).
*   **Robot Initialization**: The main robot logic and lifecycle are managed by [`Robot.java`](src/main/java/frc/robot/Robot.java).
*   **Command Bindings & Subsystem Instantiation**: All robot subsystems are instantiated and operator controls are bound to commands in [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java).

### Subsystems
*   **Mechanisms (Subsystems)**: Code related to specific robot mechanisms can be found in the `src/main/java/frc/robot/subsystems/` directory:
    - [`CommandSwerveDrivetrain.java`](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java): Field-centric swerve drive control
    - [`ShooterSubsystem.java`](src/main/java/frc/robot/subsystems/ShooterSubsystem.java): Dual motor shooter with interpolated distance-based RPS control
    - [`IntakeSubsystem.java`](src/main/java/frc/robot/subsystems/IntakeSubsystem.java): Motorized intake with flip positions
    - [`FeederSubsystem.java`](src/main/java/frc/robot/subsystems/FeederSubsystem.java): Dual feeder motors for game piece transport
    - [`WasherSubsystem.java`](src/main/java/frc/robot/subsystems/WasherSubsystem.java): Washer motor for feeding mechanism
    - [`RunRollers.java`](src/main/java/frc/robot/subsystems/RunRollers.java): Intake roller control
    - [`Limelight.java`](src/main/java/frc/robot/subsystems/Limelight.java): Vision system integration for target tracking
    - [`Superstructure.java`](src/main/java/frc/robot/subsystems/Superstructure.java): High-level coordination of multiple subsystems

### Commands & Utilities
*   **Robot Actions (Commands)**: Complex robot behaviors are defined as commands in the `src/main/java/frc/robot/commands/` directory:
    - [`AutoAlignCommand.java`](src/main/java/frc/robot/commands/AutoAlignCommand.java): General-purpose alignment to any field-centric angle using ProfiledPIDController
    - [`AutoAlignAndShoot.java`](src/main/java/frc/robot/commands/AutoAlignAndShoot.java): Align to hub and execute shooting sequence
    - [`AutoAutonAlignAndShoot.java`](src/main/java/frc/robot/commands/AutoAutonAlignAndShoot.java): Autonomous shooting aligned to specified tag
    - [`ShootLoad.java`](src/main/java/frc/robot/commands/ShootLoad.java): Queued shooting with load sequence
    - [`RunIntakeRollers.java`](src/main/java/frc/robot/commands/RunIntakeRollers.java): Timed intake roller operation

*   **Utilities**: Helper classes located in `src/main/java/frc/robot/utils/`:
    - [`Controls.java`](src/main/java/frc/robot/utils/Controls.java): Centralized controller input mapping for driver and operator
    - [`AllianceUtil.java`](src/main/java/frc/robot/utils/AllianceUtil.java): Alliance-specific calculations and targeting
    - [`LimelightHelpers.java`](src/main/java/frc/robot/utils/LimelightHelpers.java): Limelight vision processing utilities
    - [`ShooterUtil.java`](src/main/java/frc/robot/utils/ShooterUtil.java): Shooter interpolation and distance calculations

## Documentation
- [Code Layout Explanation](layout.md)
- [Shooter Subsystem Details](shooter.md)

## Build & Deployment

### Building the Project
```bash
./gradlew build
```

### Deploying to Robot
```bash
./gradlew deploy
```

### Running Simulation
```bash
./gradlew simulateJava
```

### Cleaning Build
```bash
./gradlew clean
```

## Key Features

### Autonomous System
- PathPlanner integration for path-following autonomous routines
- Vision-based auto-alignment to hub targets
- Named commands system for complex autonomous sequences
- Multiple auto routines available through SmartDashboard auto chooser

### Vision System
- Limelight camera for target tracking
- April Tag detection for field localization
- Vision odometry integration (when enabled)
- Rejection filtering based on speed and distance thresholds
- Camera pose calibration: (0.0381m, 0m, 0.5207m) with 7.5° tilt

### Drive System
- CTRE Falcon 500 motors with CANivore networking
- Swerve module configuration with drive and steer motors
- Field-centric drive with gyro stabilization
- Configurable speed multiplier (100% default)
- Auto-switching between Coast mode (disabled) and Brake mode (enabled)

### Shooter System
- Dual motor shooter with independent control
- Interpolated RPS adjustment based on hub distance
- Multiple shooting modes:
  - Fixed RPS (47.5 RPS)
  - Juggle mode (lower RPS)
  - Pass mode (specialized velocity)
  - Distance-based interpolated shots
- Washer and dual feeder coordination

### Intake System
- Motorized flip mechanism with three positions:
  - In (0°): Ready position inside chassis
  - Erect (15°): Transitional position
  - Out (120°): Floor pickup position
- Dual roller motors for game piece handling
- Manual voltage control for operator override

## Constants & Configuration

Motor IDs and key constants are defined in `Constants.java`:

### Drive Constants
- Speed multiplier: 100%
- Deadband: 0.05
- Max velocity: 0.75 rotations/second (angular)

### Shooter Constants
- Primary RPS: 47.5
- Juggle RPS: Lower speed mode
- Pass velocity: Specialized passing speed

### Intake Constants
- Flip gear ratio: 80:1
- Max velocity: 0.375 rotations/second
- Max acceleration: 3.0 rotations/second²
- PID gains: kP=80.0, kI=0.0, kD=0.0

### Camera Mounting
- Forward offset: 0.0381m
- Vertical offset: 0.5207m
- Tilt angle: 7.5°

## Troubleshooting

### Common Issues

**Build Fails**
- Ensure WPILib 2026 is installed: `wpilib-update`
- Clear Gradle cache: `./gradlew clean`
- Verify Java version: `java -version` (should be 21+)

**Deploy Fails**
- Check robot is on and network is connected
- Verify team number in `build.gradle`
- Ensure roboRIO firmware is up to date

**Vision Not Working**
- Check Limelight connection and LED status
- Verify camera IP is on correct subnet
- Confirm April Tag field layout matches competition year

**Drive Issues**
- Check all motor connections and CAN IDs
- Verify gyro is calibrated
- Confirm swerve module calibration

## Contributing

When making changes to the codebase:
1. Create a new branch from `main`
2. Make changes with clear commit messages
3. Test thoroughly on both simulation and robot
4. Submit pull request for review
5. Merge to `main` after approval

## Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [CTRE Phoenix 6 Documentation](https://api.ctr-electronics.com/phoenix6/release/)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [FRC Robotics Community](https://www.chiefdelphi.com/)

## Team Information

- **Team**: 5533 Robotics
- **Organization**: [Your school/organization name]
- **Competition Year**: 2026
- **Repository**: [Link to repo]



The robot uses dual Xbox controllers - one for the driver and one for the operator. Controls are configured in [`Controls.java`](src/main/java/frc/robot/utils/Controls.java).

### Driver Controls

| Input | Action | Notes |
|---|---|---|
| Left Stick (X/Y) | Field-centric drive translation | Deadband: 0.05, Speed multiplier: 100% |
| Right Stick (X) | Field-centric rotation | Deadband: 0.05 |
| Right Bumper (hold) | AutoAlign to hub | Requires active hub target and in alliance zone |
| Left Bumper (press) | Seed field-centric | Resets gyro heading to 0° |
| A (hold) | Swerve drive brake | Locks all wheels in X-position |
| B (hold) | Point wheels at left-stick direction | Manual wheel alignment |
| Y (hold) | Face 0 degrees | Field-centric rotation to 0° without translation |

### Operator Controls

| Input | Action | Notes |
|---|---|---|
| Left Trigger (hold) | Washer + Feeders + Shake | Feed game pieces; stops on release |
| Right Trigger (hold) | Interpolated shot | Adjustable RPS; distance-based targeting |
| Left Bumper (hold) | Intake rollers forward | Intake active; stops on release |
| Right Bumper (hold) | Intake rollers reverse | Eject game pieces; stops on release |
| A (hold) | Juggle shot | Fixed RPS for juggle mode |
| B (hold) | Intake flip out | Manual intake deployment |
| X (hold) | Intake flip manual reverse | Manual intake retraction |
| Y (hold) | Fixed RPS shoot | Shooter at fixed speed (47.5 RPS) |
| D-Pad Up (press) | Intake erect | Raises intake to vertical position |
| D-Pad Down (press) | Intake out flip | Lowers intake to floor pickup |
| D-Pad Left (hold) | Pass mode | Fixed RPS pass velocity with feeders/washer |
| D-Pad Right (hold) | Reverse shot | Reversed shooter with inverted feeders |

### Control Adjustments
- RPS (Revolutions Per Second) adjustment is available for interpolated and fixed shooting modes
- All triggers and buttons use deadband filtering for smooth control response
- Parallel commands coordinate multiple subsystems for complex actions