# Intake SysId and Tuning Guide

## Overview
- Characterize the intake flip mechanism using WPILib SysId to estimate feedforward and refine PID.
- The robot code exposes SysId commands on IntakeSubsystem for quasistatic/dynamic testing.
- The intake flip uses a trapezoidal-profile PID controller driving voltage output.

## Preconditions
- Ensure intake flip mechanism moves freely and has no mechanical interference.
- Verify TalonFX sensor units report rotations increasing in the correct direction.
- Battery charged; robot on blocks; safety observer present.
- Build and deploy code with IntakeSubsystem present.

## Commands Available
- SmartDashboard buttons are pre-added:
  - Intake Quasistatic F
  - Intake Quasistatic R
  - Intake Dynamic F
  - Intake Dynamic R

## How To Run SysId
1. Open the SysId tool in WPILib.
2. Configure data source to use NetworkTables SysId routines.
3. On SmartDashboard, click the matching Intake button (Quasistatic/Dynamic, Forward/Reverse).
4. Press run in SysId while the command is scheduled.
5. Collect forward and reverse datasets.

## Data Notes
- SysId logs voltage, angular position (rotations), and angular velocity (rot/s).
- Quasistatic tests estimate kS and kV; dynamic adds kA.

## Parameter Fitting
- Use SysId’s fit to extract kS, kV, kA.
- For trapezoid PID on intake flip:
  - Use kS, kV, kA only if you incorporate feedforward. Current code uses PID-only voltage; feedforward can be added later.
  - Tune PID:
    - Increase flipkP until step commands move crisply without significant overshoot.
    - Add small flipkD if oscillation occurs.
    - Keep flipkI at 0 unless you see steady-state error.
- Constraints:
  - flipMaxVelocityRotPerS and flipMaxAccelRotPerSSq set the trapezoid profile aggressiveness. Reduce if oscillation or brownouts occur.

## Plugging Values In
- Edit Constants.IntakeConstants:
  - flipkP, flipkI, flipkD
  - flipMaxVelocityRotPerS, flipMaxAccelRotPerSSq
  - rollerVoltage if needed.

## Test Procedure
- Command setFlip(In) and setFlip(Out) repeatedly.
- Verify motion is smooth, repeatable, and holds position.
- Validate roller forward/reverse voltages for appropriate current draw without brownouts.

## Safety
- Always run on blocks for characterization.
- Clamp voltage outputs as needed to protect hardware.
