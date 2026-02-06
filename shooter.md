# Shooter Align-and-Shoot

## Operator Y Behavior

- Hold Y: robot auto-aligns to hub.
- When aligned: shooters spin to velocity based on distance, washers feed.
- Release Y: shooters and washers stop.

## Velocity Control

- Mode: VelocityVoltage closed-loop per motor in rotations per second.
- Gains: kP, kI, kD, kV from Constants.ShooterConstants.

## SysId Characterization

- SmartDashboard has buttons for Shooter Left and Shooter Right:
  - Quasistatic Forward/Reverse, Dynamic Forward/Reverse.
- Procedure:
  - Remove game pieces, disable washers.
  - Run Quasistatic F/R on one side; save data in SysId tool; repeat Dynamic F/R.
  - Fit kS, kV, kA in the SysId analyzer for each side.
  - Update Constants.ShooterConstants with the feedforward terms; keep P/D small at first.
  - Validate by commanding a setpoint and checking RPS vs setpoint on dashboard.

## Tuning Strategy

- Start with feedforward: set kV to match steady-state velocity with minimal error.
- Add kS if startup friction causes underspeed at low setpoints.
- Add P to remove residual steady-state error; increase until response is crisp but not oscillatory.
- Add D to damp overshoot if transients ring; keep I near zero unless persistent bias remains.
- Clamp washer voltage to avoid jams; adjust only after shooter velocity holds.

## Verification

- Dashboard shows Shooter/LeftRPS, Shooter/RightRPS, and setpoints.
- Step test: command a velocity, verify measured RPS reaches and holds setpoint quickly.
- Distance shots: stand at known distances, hold Y, confirm balls hit target consistently.
 - Use SysId plots to confirm linearity and identify saturation; adjust map points if far-range drops.

## Building the Distance→Velocity Map

- Use known field marks to gather pairs: distance meters → shooter RPS.
- Populate points around key ranges (close, mid, far). The system interpolates between points.
- Iterate during practice: refine points where shots miss high/low.

## Notes

- Alignment uses Limelight tags when available; falls back to odometry.
- Shooter runs only after aligned; keep the robot stationary during fire for consistency.
