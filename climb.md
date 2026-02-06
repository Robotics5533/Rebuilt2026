# Climb SysId and Tuning Guide

## Overview
- Characterize the climb mechanism with WPILib SysId to estimate feedforward terms.
- Use MotionMagic for position control with cruise velocity and acceleration.
- The robot code keeps the “Hanging” setpoint applied even when disabled.

## Preconditions
- Mechanism secure; robot on blocks or stand; safety observer present.
- TalonFX encoder reports rotations correctly and is zeroed to a known position.
- Build and deploy with ClimbSubsystem present.

## Commands Available
- SmartDashboard buttons are pre-added:
  - Climb Quasistatic F
  - Climb Quasistatic R
  - Climb Dynamic F
  - Climb Dynamic R

## How To Run SysId
1. Open WPILib SysId tool.
2. Configure data source to use NetworkTables SysId routines.
3. On SmartDashboard, click the matching Climb button (Quasistatic/Dynamic, Forward/Reverse).
4. Press run in SysId while the command is scheduled.
5. Collect forward and reverse datasets.

## Parameter Fitting
- Use SysId fit to obtain kS, kV, kA.
- MotionMagic recommended slot gains:
  - kS (static) and kV (velocity) improve tracking with cruise velocity targets.
  - kA may be used to better handle acceleration.
  - kP provides position stiffness; start moderate and increase until steady tracking with minimal overshoot.
  - kD can damp oscillations if needed; keep small.
  - kI typically 0; add only for steady-state error.

## MotionMagic Settings
- Cruise velocity and acceleration:
  - Start with conservative values (e.g., 80 rps cruise, 160 rps/s accel).
  - Increase gradually if mechanism is stable and power is sufficient.

## Plugging Values In
- Edit Constants.ClimbConstants:
  - kP, kI, kD; consider adding kS, kV, kA if you incorporate feedforward.
  - inactivePositionRot, activePositionRot, hangingPositionRot.
- Update MotionMagicCruiseVelocity and MotionMagicAcceleration in the subsystem configuration.

## Operational Notes
- Operator B cycles climb state: Inactive → Active → Hanging → Inactive.
- RobotContainer applies setpoints ignoringDisable, so Hanging holds after match end.

## Safety
- Keep the mechanism within soft limits during SysId.
- Monitor current draw and stop immediately on unusual sounds or spikes.
