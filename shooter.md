# Shooter Tuning Guide (Manual Method)

## 1. Find Feedforward (kV)
*This is the most critical step. If kV is wrong, the PID will struggle to compensate.*

1.  **Preparation**:
    *   Set `kP`, `kI`, `kD` to `0` in `Constants.java`.
    *   Ensure the robot is on a cart or blocks so wheels can spin freely.
    *   Deploy the code.
2.  **Test**:
    *   Command the shooter to run at a constant **Voltage** (e.g., 6 Volts).
    *   Observe the stable velocity on SmartDashboard (e.g., `Shooter/LeftRPS`).
3.  **Calculate**:
    *   Formula: `kV = Volts / RPS`
    *   *Example*: If 6 Volts results in 50 RPS:
        *   `6.0 / 50.0 = 0.12`
    *   Set this `kV` value in `Constants.java`.

## 2. Find Feedforward Static (kS)
*This overcomes static friction to get the wheel moving.*

1.  Slowly increase the voltage command from 0.0V.
2.  Note the voltage where the flywheel *just* begins to spin reliably.
3.  This is your `kS`. It is typically between **0.1V - 0.5V**.

## 3. Tune Proportional (kP)
*This handles disturbance rejection (recovery after a shot).*

1.  Set a realistic target velocity (e.g., **80 RPS**).
2.  With `kV` set, the wheel should spin up close to 80 RPS but may "sag" slightly or drop significantly when a game piece is fired.
3.  Increase `kP` in small increments (start at `0.05`, then `0.1`, `0.2`, etc.).
4.  **Goal**: When a ball is fired, the RPM will dip. A properly tuned `kP` forces the motor to apply max voltage immediately to recover back to 80 RPS.
    *   **Too Low**: Recovery is slow; subsequent shots in a rapid-fire sequence will be weak (undershoot).
    *   **Too High**: The flywheel oscillates (audible humming/vibration) or overshoots the target.

## 4. Verification
1.  Open SmartDashboard/Shuffleboard.
2.  Graph `Shooter/LeftRPS` vs `Shooter/LeftSetpointRPS`.
3.  The actual RPS line should track the Setpoint line tightly during spin-up and overlap almost perfectly during steady state.
