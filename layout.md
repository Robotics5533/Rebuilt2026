# Codebase Layout

This document outlines the general structure and purpose of key files and folders within the robot's codebase, based on the WPILib Command-Based framework.

## Top-Level Directories

*   `src/main/java/frc/robot/`: This directory contains all the main Java source code for the robot. It follows the standard WPILib project structure.
*   `README.md`: Provides general information about the project, including current control mappings and setup instructions.

## Core Robot Files (frc/robot/)

*   [`Robot.java`](src/main/java/frc/robot/Robot.java):
    - The main robot class that extends `TimedRobot`.
    - Handles the robot's lifecycle, including initialization (robotInit), periodic loops for different modes (teleopPeriodic, autonomousPeriodic, testPeriodic), and disabled states.
    - Instantiates the `RobotContainer`.

*   [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java):
    - The central place for instantiating all robot subsystems and defining button bindings for operator controls.
    - Connects driver and operator input to specific commands.
    - Manages autonomous command selection.

*   [`Constants.java`](src/main/java/frc/robot/Constants.java):
    - Stores all global constants, including:
        - Motor CAN IDs.
        - PID gains for various mechanisms.
        - Physical constraints (e.g., gear ratios, wheel diameters).
        - Controller deadbands and trigger thresholds.
        - Autonomous routine parameters.
    - Organized into nested classes (e.g., `DriveConstants`, `ShooterConstants`, `OperatorConstants`) for clarity.

## Subsystems (frc/robot/subsystems/)

This package contains classes that represent individual mechanisms or functional units of the robot. Each subsystem owns its hardware (motors, sensors) and exposes methods for controlling them and querying their state. Subsystems should generally not contain direct command logic.

*   [`ShooterSubsystem.java`](src/main/java/frc/robot/subsystems/ShooterSubsystem.java):
    - Controls the shooter motors (e.g., `TalonFX`).
    - Provides methods for setting shooter speed/voltage, checking if at target speed, and command factories for running/stopping the shooter.

*   [`FeederSubsystem.java`](src/main/java/frc/robot/subsystems/FeederSubsystem.java):
    - Manages the feeder motors responsible for moving game pieces into the shooter.
    - Offers methods for running and stopping feeder motors at specified voltages or speeds, and corresponding command factories.

*   [`IntakeSubsystem.java`](src/main/java/frc/robot/subsystems/IntakeSubsystem.java):
    - Controls the robot's intake mechanism.
    - Provides methods for running the intake rollers (forward/reverse), flipping the intake, and related command factories.

*   [`WasherSubsystem.java`](src/main/java/frc/robot/subsystems/WasherSubsystem.java):
    - Manages the washer mechanism, typically used for cleaning or preparing game pieces.
    - Contains methods for running the washer at a specific voltage and stopping it, along with command factories.

*   [`CommandSwerveDrivetrain.java`](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java):
    - Represents the robot's swerve drive base.
    - Handles all aspects of swerve control, including odometry, field-centric driving, and module state management.

## Commands (frc/robot/commands/)

This package contains classes that define robot actions. Commands orchestrate one or more subsystems to achieve a specific task. They describe *what* the robot should do, while subsystems describe *how* to do it.

*   [`ShootLoad.java`](src/main/java/frc/robot/commands/ShootLoad.java):
    - A complex command that sequences the shooter, washer, and feeder mechanisms.
    - First, brings the shooters up to speed, then runs the washer and feeders in parallel for a specified duration, finally stopping all three.

*   [`AutoAlignHub.java`](src/main/java/frc/robot/commands/AutoAlignHub.java):
    - A command responsible for automatically aligning the robot with a target (e.g., the speaker/hub) using vision data (e.g., from a Limelight).
    - Integrates with the drivetrain to adjust robot position and orientation.

## Utilities (frc/robot/utils/)

This package contains general utility classes that support various aspects of the robot's functionality but don't fit directly into subsystems or commands.

*   [`Controls.java`](src/main/java/frc/robot/utils/Controls.java):
    - Centralizes the configuration of driver and operator controllers.
    - Maps controller inputs (buttons, triggers, joysticks) to specific robot commands.
    - Includes methods for setting controller rumble feedback.
