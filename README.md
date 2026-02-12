# Rebuilt2026 Robot Code

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
| Right Trigger (hold) | Right shooter run + washer | Threshold 0.5 |
| Left Trigger (hold) | Left shooter run + washer | Threshold 0.5 |
| X (hold) | Intake flip manual forward (3V) | Calibration jog |
| Back (hold) | Intake flip manual reverse (3V) | Calibration jog |

Notes:
- Intake flip calibration logs Intake/CapturedFlipOutRot continuously during manual jog.
- PathPlanner named command "shoot_load" is registered and will shoot until match time ≤ 5.5s.
 - Shooter uses distance→velocity interpolation when firing via Y; see shooter.md.

## Mechanism Conversions

- Climb rotationsToInches (inches per motor rotation):
  - Drum: (PI × drumDiameterInches) ÷ motorToDrumGearRatio
  - Leadscrew: (leadPitchInchesPerRev) ÷ motorToScrewGearRatio

## SysId Buttons

- SmartDashboard provides buttons for Intake and Climb SysId:
  - Intake Quasistatic F/R, Intake Dynamic F/R
  - Climb Quasistatic F/R, Climb Dynamic F/R
  - Shooter Left Quasistatic F/R, Shooter Left Dynamic F/R
  - Shooter Right Quasistatic F/R, Shooter Right Dynamic F/R

## Simulation

- Enabled WPILib simulation GUI with driver station.
- Visualizations:
  - ShooterViz: two ligaments show left/right flywheel speed.
  - IntakeViz: arm ligament shows intake flip angle.
- Run sim: Gradle sim tasks launch GUI; use controller to exercise bindings.
