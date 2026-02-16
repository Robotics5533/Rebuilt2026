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
| Left Trigger (hold) | Run washer and both feeders | Threshold 0.5 |
| Right Trigger (hold) | Run both shooters to speed | Threshold 0.5 |
| X (hold) | Intake flip manual forward (3V) | Calibration jog |
| Back (hold) | Intake flip manual reverse (3V) | Calibration jog |