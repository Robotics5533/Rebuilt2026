# Rebuilt2026 Robot Code

This repository contains the official software for the Rebuilt2026 FRC robot for team 5533, developed using the WPILib Command-Based framework. This code is engineered for robust performance in competition, emphasizing reliability, debuggability, and adherence to best practices in FRC software development.

## Getting Started

To get started with the codebase, ensure you have WPILib (2026 or newer) installed and configured for Java development. Clone this repository and open it in VS Code.

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
| Right Bumper (hold) | AutoAlign to hub | Conditional on active hub trigger and being in alliance zone |
| Left Bumper (press) | Seed field-centric | Resets field-centric direction |
| A (hold) | Swerve drive brake | Holds position |
| B (hold) | Point wheels at left-stick direction | Wheel align |
| Y (hold) | Face 0 degrees | Robot will orient to 0 degrees field-centric |
| D-pad Up (hold) | Manual climb up | |
| D-pad Down (hold) | Manual climb down | |

### Operator Controls

| Input | Action | Notes |
|---|---|---|
| Left Trigger (hold) | Feed/washer | Runs washer and both feeders |
| Right Trigger (hold) | Shooting | Runs interpolated shot |
| Left Bumper (hold) | Intake power out | Stops on release |
| Right Bumper (hold) | Intake power in | Stops on release |
| B (press) | Intake out position | Sets intake to out position |
| X (press) | Intake in position | Sets intake to in position |
| Y (hold) | Shooter at 47.5 RPS | Fixed speed shooting, not interpolated |
| D-pad Left (hold) | AutoAlign and Shoot | Aligns to hub, spins up shooters, then shoots once aligned and at speed. Conditional on being in alliance zone |
| D-pad Up (press) | Increment shooter RPS adjustment | Adjusts target RPS for interpolated and fixed shots |
| D-pad Down (press) | Decrement shooter RPS adjustment | Adjusts target RPS for interpolated and fixed shots |