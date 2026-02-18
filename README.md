# Rebuilt2026 Robot Code

This repository contains the official software for the Rebuilt2026 FRC robot for team 5533, developed using the WPILib Command-Based framework. This code is engineered for robust performance in competition, emphasizing reliability, debuggability, and adherence to best practices in FRC software development.

## Getting Started

To get started with the codebase, ensure you have WPILib (2024 or newer) installed and configured for Java development. Clone this repository and open it in VS Code.

## Codebase Navigation

The robot's software is structured according to the WPILib Command-Based paradigm, promoting modularity and clear separation of concerns.

*   **Core Structure**: The primary Java source code resides in `src/main/java/frc/robot/`.
*   **Central Configuration**: Global constants, motor IDs, PID gains, and physical parameters are defined in [`Constants.java`](Constants.java).
*   **Robot Initialization**: The main robot logic and lifecycle are managed by [`Robot.java`](Robot.java).
*   **Command Bindings & Subsystem Instantiation**: All robot subsystems are instantiated and operator controls are bound to commands in [`RobotContainer.java`](RobotContainer.java).
*   **Mechanisms (Subsystems)**: Code related to specific robot mechanisms (e.g., Shooter, Drivetrain, Intake) can be found in the `src/main/java/frc/robot/subsystems/` directory. Each subsystem manages its hardware and exposes control methods.
*   **Robot Actions (Commands)**: Complex robot behaviors and actions are defined as commands in the `src/main/java/frc/robot/commands/` directory. Commands orchestrate subsystems to perform tasks.
*   **Utilities**: Helper classes, such as controller mappings and rumble feedback, are located in `src/main/java/frc/robot/utils/`. For a detailed explanation of the code layout, refer to the [Code Layout Explanation](layout.md).

## Documentation
- [Code Layout Explanation](layout.md)
- [Shooter Subsystem Details](shooter.md)

## Controls

### Driver Controls

| Input | Action | Notes |
|---|---|---|
| Left Stick (X/Y) | Field-centric drive translation | Deadband applied |
| Right Stick (X) | Field-centric rotation | Deadband applied |
| Right Bumper (hold) | AutoAlign to hub | Uses Limelight+PID |
| Left Bumper (press) | Seed field-centric | Sets operator perspective |
| A (hold) | Swerve drive brake | Hold position |
| B (hold) | Point wheels at left-stick direction | Wheel align |

### Operator Controls

| Input | Action | Notes |
|---|---|---|
| A (press) | Intake flip toggle | Blocks Out when climb engaged |
| Right Bumper (hold) | Intake roller forward | Stops on release |
| Left Bumper (hold) | Intake roller reverse | Stops on release |
| Left Trigger (hold) | Run washer and both feeders | Threshold 0.5 |
| Right Trigger (hold) | Run both shooters to speed | Threshold 0.5 |
| X (hold) | Intake flip manual forward (3V) | Calibration jog |
| Back (hold) | Intake flip manual reverse (3V) | Calibration jog |