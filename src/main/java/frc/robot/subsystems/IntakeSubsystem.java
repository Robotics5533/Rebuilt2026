package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

/**
 * ============================================================================
 * IntakeSubsystem - Handles the flip mechanism for the intake
 * ============================================================================
 * 
 * MOTOR CONFIGURATION (LEADER-FOLLOWER SYSTEM):
 * - LEFT MOTOR:  The "LEADER" - This motor receives all commands (PID control)
 * - RIGHT MOTOR: The "FOLLOWER" - This motor automatically copies the left motor
 * 
 * WHY USE LEADER-FOLLOWER?
 * When you have two motors working together (like a flip mechanism), they need
 * to move in sync. The leader-follower system means we only need to send commands
 * to ONE motor (the leader), and the other motor (the follower) automatically
 * copies what the leader does. This avoids the problem of both motors running
 * their own PID controller and fighting each other.
 * 
 * MOTOR ALIGNMENT - UNDERSTANDING "Opposed":
 * MotorAlignmentValue.Opposed means the right motor will rotate in the OPPOSITE
 * direction compared to the left motor's output. This is used when motors are
 * physically mounted upside-down or facing opposite directions but need to move
 * the mechanism together.
 * 
 * TO CHANGE THE RIGHT MOTOR BEHAVIOR:
 * Look for: MotorAlignmentValue.Opposed
 * - Keep it as "Opposed" if motors need to rotate in opposite directions
 * - Change to "Aligned" if motors should rotate in the same direction
 * ============================================================================
 */
public class IntakeSubsystem extends SubsystemBase {
  
  // ============================================================================
  // FlipState ENUM - Defines the different positions the intake can be in
  // ============================================================================
  // An enum is like a list of predefined options. The intake can only be in
  // one of these three positions. Each position has an angle (in degrees).
  public enum FlipState {
    In(Constants.IntakeConstants.flipInPositionDeg),      // Intake pulled into robot
    Out(Constants.IntakeConstants.flipOutPositionDeg),    // Intake extended out
    Erect(Constants.IntakeConstants.flipErectPositionDeg); // Intake vertical

    // This stores the angle (in degrees) for each position
    public final double angleDeg;

    // Constructor - runs when each FlipState is created
    FlipState(double angleDeg) {
      this.angleDeg = angleDeg;
    }

    // Convert the angle from degrees to WPILib's Angle units
    public Angle angle() {
      return Degrees.of(angleDeg);
    }
  }

  // ============================================================================
  // MOTOR DECLARATIONS - These create the motor objects we'll control
  // ============================================================================
  // leftFlipMotor:  This motor receives the PID-controlled commands (LEADER)
  // rightFlipMotor: This motor follows the left motor automatically (FOLLOWER)
  // 
  // The number inside (e.g., intakeLeftFlipMotorId) is the CAN ID that identifies
  // this specific motor on the CAN bus network of the robot.
  private final TalonFX leftFlipMotor = new TalonFX(Constants.IntakeConstants.intakeLeftFlipMotorId);
  private final TalonFX rightFlipMotor = new TalonFX(Constants.IntakeConstants.intakeRightFlipMotorId);

  // ============================================================================
  // VOLTAGE CONTROL OBJECT - Used to send voltage commands to motors
  // ============================================================================
  // VoltageOut is like a "template" for commands. We use it to send voltage
  // values (between -12V and +12V) to the motors. Positive voltage = one direction,
  // Negative voltage = opposite direction.
  private final VoltageOut flipVoltageCtrl = new VoltageOut(0);

  // ============================================================================
  // PID CONTROLLER - Smooth motion and position control
  // ============================================================================
  // PID (Proportional-Integral-Derivative) is an algorithm that smoothly moves
  // the motor from one position to another without jerking.
  // 
  // WHAT IT DOES:
  // 1. We tell it a target position (e.g., "go to 45 degrees")
  // 2. We tell it the current position (from the motor sensor)
  // 3. It calculates how much voltage to send to reach the target smoothly
  // 4. It automatically adjusts the voltage if we're overshooting or undershooting
  // 
  // The kP, kI, kD values are "tuning parameters" - they control how aggressive
  // the PID controller is. These values are in Constants.java and can be adjusted.
  // 
  // TrapezoidProfile creates acceleration curves so the motor doesn't jump
  // instantly to full speed - it accelerates smoothly.
  private final ProfiledPIDController flipController = new ProfiledPIDController(
      Constants.IntakeConstants.flipkP,           // Proportional gain (main control)
      Constants.IntakeConstants.flipkI,           // Integral gain (steady-state correction)
      Constants.IntakeConstants.flipkD,           // Derivative gain (damping/smoothing)
      new TrapezoidProfile.Constraints(
          Constants.IntakeConstants.flipMaxVelocityRotPerS,    // Max speed (rotations/sec)
          Constants.IntakeConstants.flipMaxAccelRotPerSSq));   // Max acceleration (rotations/sec²)

  // ============================================================================
  // STATE VARIABLES - Track what's happening now
  // ============================================================================
  // flipState: What position do we want to be in? (In, Out, or Erect)
  private FlipState flipState = FlipState.In;
  
  // interlockEnabled: When true, prevents the intake from flipping out 
  // (safety feature - prevents damage if something goes wrong)
  private boolean interlockEnabled = false;
  
  // capturedManualPosition: When someone is manually controlling the intake,
  // this stores the position where they stopped, so it "holds" there.
  // null = not in manual mode
  private Double capturedManualPosition = null;

  // ============================================================================
  // CONSTRUCTOR - Runs once when the robot starts
  // ============================================================================
  public IntakeSubsystem() {
    // Create a configuration object that will be applied to both motors
    // This sets up safety limits, current limits, and other settings.
    var baseCfg = new TalonFXConfiguration();
    
    // Configure feedback (position/velocity sensing)
    baseCfg.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.flipGearRatio;
    
    // Configure current limits (prevents motors from drawing too much power)
    // Supply current = power from battery, Stator current = power in motor
    baseCfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.flipCurrentLimit;
    baseCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    baseCfg.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.flipCurrentLimit;
    baseCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Configure software limits (prevents motor from going beyond safe angles)
    // These act like virtual "bumpers" to stop the motor at extreme positions
    baseCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
        .degreesToRotations(Constants.IntakeConstants.softLimitForwardDeg);
    baseCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    baseCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
        .degreesToRotations(Constants.IntakeConstants.softLimitReverseDeg);
    baseCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // ========================================================================
    // CONFIGURE LEFT MOTOR (LEADER)
    // ========================================================================
    leftFlipMotor.getConfigurator().apply(baseCfg);           // Apply our configuration
    leftFlipMotor.setNeutralMode(NeutralModeValue.Brake);     // Brake keeps it in place when power is 0
    leftFlipMotor.setPosition(0);                             // Reset position counter to 0

    // ========================================================================
    // CONFIGURE RIGHT MOTOR (FOLLOWER)
    // ========================================================================
    rightFlipMotor.getConfigurator().apply(baseCfg);          // Apply our configuration
    rightFlipMotor.setNeutralMode(NeutralModeValue.Brake);    // Brake keeps it in place when power is 0
    rightFlipMotor.setPosition(0);                            // Reset position counter to 0

    // ========================================================================
    // INITIALIZE STARTING POSITION (Both motors start in "In" position)
    // ========================================================================
    // This assumes the robot starts with the intake in the "In" (retracted) position
    double inPosRot = Units.degreesToRotations(Constants.IntakeConstants.flipInPositionDeg);
    leftFlipMotor.setPosition(inPosRot);
    rightFlipMotor.setPosition(inPosRot);

    // ========================================================================
    // INITIALIZE PID CONTROLLER
    // ========================================================================
    // Tell the PID controller: "We're starting at this position"
    // This so it knows the starting point for calculating smooth motion
    flipController.reset(inPosRot);
    flipController.setTolerance(Units.degreesToRotations(Constants.IntakeConstants.flipToleranceDeg));
  }

  // ============================================================================
  // setFlip() - Change the desired flip position
  // ============================================================================
  // This is called when we want to move to a different position (In, Out, Erect)
  public void setFlip(FlipState s) {
    // Safety feature: If interlock is enabled AND we're trying to flip out,
    // just go to "In" instead to prevent damage
    if (interlockEnabled && s == FlipState.Out) {
      flipState = FlipState.In;
      return;
    }

    // Update the desired position
    flipState = s;
    
    // Clear manual position capture so we go back to automatic control
    capturedManualPosition = null;
  }

  // ============================================================================
  // toggleFlip() - Switch between In and Out positions
  // ============================================================================
  // Easy method to flip back and forth without specifying exact positions
  public void toggleFlip() {
    // If we're "In", go to "Out". If we're "Out", go to "In"
    setFlip(flipState == FlipState.In ? FlipState.Out : FlipState.In);
  }

  // ============================================================================
  // erect() - Move to the Erect (vertical) position
  // ============================================================================
  public void erect() {
    setFlip(FlipState.Erect);
  }

  // ============================================================================
  // outFlip() - Move to the Out (extended) position
  // ============================================================================
  public void outFlip() {
    setFlip(FlipState.Out);
  }

  // ============================================================================
  // flipAtTarget() - Check if we've reached our goal position
  // ============================================================================
  // Returns true if current position is within tolerance of target position
  // Used to know when movement is complete
  private boolean flipAtTarget() {
    double currentRot = getFlipPositionRot();           // Where are we now?
    double targetRot = Units.degreesToRotations(flipState.angleDeg); // Where do we want to be?
    
    // Check if the difference is small enough (within tolerance)
    // tolerance = how close we need to be to consider it "at target"
    return Math.abs(currentRot - targetRot) <= Units.degreesToRotations(Constants.IntakeConstants.flipToleranceDeg);
  }

  // ============================================================================
  // getFlipPositionRot() - Get current motor position in ROTATIONS
  // ============================================================================
  // This reads from the LEFT MOTOR only (the leader)
  // Why? Because the right motor follows the left, so they should always be
  // at the same position. We only need to read one.
  private double getFlipPositionRot() {
    return leftFlipMotor.getPosition().getValueAsDouble();
  }

  // ============================================================================
  // getFlipPositionDeg() - Get current motor position in DEGREES
  // ============================================================================
  // Converts rotations to degrees (more readable for humans)
  public double getFlipPositionDeg() {
    return Units.rotationsToDegrees(getFlipPositionRot());
  }

  // ============================================================================
  // periodic() - MAIN CONTROL LOOP (Runs every cycle ~20ms)
  // ============================================================================
  // This is called automatically by WPILib many times per second.
  // This is where the actual control happens - we read sensors, calculate
  // what to do, and send commands to motors.
  @Override
  public void periodic() {
    // ========================================================================
    // STEP 1: Read current position from the left motor
    // ========================================================================
    double currentRot = getFlipPositionRot();

    // ========================================================================
    // STEP 2: Determine the target position
    // ========================================================================
    // If someone is manually controlling (capturedManualPosition != null),
    // use that. Otherwise use the desired flipState position.
    double targetRot = (capturedManualPosition != null)
        ? capturedManualPosition
        : Units.degreesToRotations(flipState.angleDeg);

    // ========================================================================
    // STEP 3: Use PID controller to calculate smooth voltage output
    // ========================================================================
    // The PID controller takes:
    //   - Current position (where we are now)
    //   - Target position (where we want to go)
    // And returns a voltage value that will smoothly move us to the target
    double pidOutput = flipController.calculate(currentRot, targetRot);

    // ========================================================================
    // STEP 4: Add gravity assist to help lift the mechanism
    // ========================================================================
    // Gravity pulls down on the intake, so we need extra voltage to counteract it
    // This "gravity assist" voltage is added to help the motor lift against gravity
    double gravityAssist = Constants.IntakeConstants.flipGravityAssistVoltage;

    // ========================================================================
    // STEP 5: Combine PID output and gravity assist
    // ========================================================================
    // Total voltage = smooth motion control + gravity compensation
    double totalVoltage = pidOutput + gravityAssist;

    // ========================================================================
    // STEP 6: Send voltage command to LEFT MOTOR (the leader)
    // ========================================================================
    // Negative voltage makes it go counter-clockwise
    // Positive voltage makes it go clockwise
    // By sending -totalVoltage, when totalVoltage is positive, left motor goes CCW
    leftFlipMotor.setControl(flipVoltageCtrl.withOutput(-totalVoltage));
    
    // ========================================================================
    // STEP 7: RIGHT MOTOR follows the LEFT MOTOR automatically
    // ========================================================================
    // The Follower control tells the right motor:
    // "Copy what the left motor is doing with MotorAlignmentValue.Opposed"
    // 
    // **IMPORTANT**: Where to change motor behavior:
    // This line controls how the right motor moves:
    //   - MotorAlignmentValue.Opposed   = Right motor rotates OPPOSITE to left motor
    //   - MotorAlignmentValue.Aligned   = Right motor rotates SAME direction as left motor
    // 
    // If motors are moving the wrong way relative to each other, change
    // "Opposed" to "Aligned" (or vice versa) in BOTH places this line appears
    // (here in periodic() and in setFlipManualVoltage())
    rightFlipMotor.setControl(new Follower(leftFlipMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // ========================================================================
    // STEP 8: Send data to SmartDashboard for debugging/monitoring
    // ========================================================================
    // SmartDashboard is a dashboard on the driver station that shows live data
    // These lines send our important values so we can see what's happening:
    SmartDashboard.putNumber("Intake/FlipDeg", Units.rotationsToDegrees(currentRot));
    SmartDashboard.putNumber("Intake/FlipGoalDeg",
        capturedManualPosition != null ? Units.rotationsToDegrees(capturedManualPosition) : flipState.angleDeg);
    SmartDashboard.putNumber("Intake/FlipAppliedVolts", totalVoltage);
    SmartDashboard.putBoolean("Intake/FlipAtTarget", flipAtTarget());
  }

  // ============================================================================
  // setInterlockEnabled() - Enable/disable safety interlock
  // ============================================================================
  // When enabled, prevents the intake from flipping out (safety feature)
  public void setInterlockEnabled(boolean enabled) {
    this.interlockEnabled = enabled;
  }

  // ============================================================================
  // setFlipManualVoltage() - Allow manual voltage control
  // ============================================================================
  // This lets joystick input or other code directly control the intake
  // without using the automatic position controller
  public void setFlipManualVoltage(double volts) {
    // Send voltage directly to left motor
    // Negative = counter-clockwise, Positive = clockwise
    leftFlipMotor.setControl(flipVoltageCtrl.withOutput(-volts));
    
    // Right motor follows the left motor
    // **IMPORTANT**: This is the second place to change motor alignment
    // See the explanation in periodic() method above
    rightFlipMotor.setControl(new Follower(leftFlipMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  // ============================================================================
  // stopFlipManual() - Stop manual control and hold position
  // ============================================================================
  // When manual joystick control ends, this captures where the intake is
  // and holds it there (instead of shooting back to flipState position)
  public void stopFlipManual() {
    // Remember where we stopped
    double currentRot = getFlipPositionRot();
    capturedManualPosition = currentRot;
    
    // Reset the PID controller to know this is our new position
    flipController.reset(currentRot);
    flipController.setGoal(new TrapezoidProfile.State(currentRot, 0));
  }

  // ============================================================================
  // flipManualForwardCommand() - Create a command for manual forward motion
  // ============================================================================
  // This is called from RobotContainer when someone holds the joystick forward.
  // The command continuously calls setFlipManualVoltage() while the joystick
  // is held, and calls stopFlipManual() when released.
  public Command flipManualForwardCommand(double volts) {
    return run(() -> setFlipManualVoltage(Math.max(0.0, Math.min(Constants.IntakeConstants.manualFlipVoltage, volts))))
        .finallyDo(this::stopFlipManual);
  }

  // ============================================================================
  // flipManualReverseCommand() - Create a command for manual reverse motion
  // ============================================================================
  // Same as forward, but sends negative voltage (opposite direction)
  public Command flipManualReverseCommand(double volts) {
    return run(() -> setFlipManualVoltage(-Math.max(0.0, Math.min(Constants.IntakeConstants.manualFlipVoltage, volts))))
        .finallyDo(this::stopFlipManual);
  }

  // ============================================================================
  // captureFlipOutRotCommand() - Record the "Out" position angle
  // ============================================================================
  // This is a debugging tool - it captures the current angle and sends it
  // to SmartDashboard so engineers can see what angle the intake reaches
  // when fully extended
  public Command captureFlipOutRotCommand() {
    return runOnce(() -> SmartDashboard.putNumber("Intake/CapturedFlipOutDeg", getFlipPositionDeg()));
  }
}