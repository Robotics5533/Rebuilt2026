package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.AllianceUtil;
import java.util.function.DoubleSupplier;

/**
 * Represents the robot's shooter mechanism, controlling two Falcon 500 (TalonFX) motors.
 * This subsystem manages both voltage-based and velocity-based control of the shooters,
 * along with methods for checking speed, stopping, and generating commands for control.
 */
public class ShooterSubsystem extends SubsystemBase {

  // TalonFX motor controllers for the left and right shooter wheels.
  private final TalonFX leftShooter = new TalonFX(Constants.ShooterConstants.leftShooterId);
  private final TalonFX rightShooter = new TalonFX(Constants.ShooterConstants.rightShooterId);

  // Control requests for the TalonFXs. These are reused to reduce object allocation.
  private final VelocityVoltage velocityCtrl = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageCtrl = new VoltageOut(0);

  // Stores the last commanded setpoints for SmartDashboard telemetry.
  private double lastLeftSetpointRps = 0.0;
  private double lastRightSetpointRps = 0.0;

  /**
   * Defines the side of the shooter (Left, Right, or Both).
   */
  public enum ShooterSide {
    LEFT,
    RIGHT,
    BOTH
  }

  /**
   * Defines the control mode for the shooters (Voltage or Velocity).
   */
  public enum ShooterControlMode {
    VOLTAGE,
    VELOCITY
  }

  // The current control mode for the shooters, defaults to VELOCITY.
  private ShooterControlMode controlMode = ShooterControlMode.VELOCITY;

  // The LimelightSubsystem instance for vision processing.
  private final CommandSwerveDrivetrain drivetrain;
  private final String limelightName;

  /**
   * Constructs a new ShooterSubsystem.
   * Configures the TalonFX motor controllers with PID gains, current limits, and neutral mode.
   * @param drivetrain The CommandSwerveDrivetrain instance for odometry.
   * @param limelightName The name of the Limelight camera.
   */
  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
    this.drivetrain = drivetrain;
    this.limelightName = limelightName;
    // Create a new configuration object for the TalonFXs.
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Configure PID gains for velocity control (Slot 0).
    cfg.Slot0 = new Slot0Configs()
        .withKP(Constants.ShooterConstants.kP)
        .withKI(Constants.ShooterConstants.kI)
        .withKD(Constants.ShooterConstants.kD)
        .withKV(Constants.ShooterConstants.kV);

    // Configure current limits to prevent motor damage.
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.shooterCurrentLimit;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.shooterCurrentLimit;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    // Apply the configurations to both shooter motors.
    leftShooter.getConfigurator().apply(cfg);
    rightShooter.getConfigurator().apply(cfg);

    // Set the neutral mode to Coast, allowing the motors to spin freely when idle.
    leftShooter.setNeutralMode(NeutralModeValue.Coast);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Sets the control mode for the shooter motors.
   * @param mode The desired control mode (VOLTAGE or VELOCITY).
   */
  public void setControlMode(ShooterControlMode mode) {
    this.controlMode = mode;
  }

  /**
   * Checks if a specific shooter side (or both) has reached its target speed.
   * @param side The shooter side to check (LEFT, RIGHT).
   * @return True if the specified shooter(s) are at or above the speed threshold, false otherwise.
   */
  public boolean isAtSpeed(ShooterSide side) {
    switch (side) {
      case LEFT:
        // Check if the absolute velocity of the left shooter is at or above the threshold.
        return Math
            .abs(leftShooter.getVelocity().getValueAsDouble()) >= Constants.ShooterConstants.shooterSpeedThresholdRPS;
      case RIGHT:
        // Check if the absolute velocity of the right shooter is at or above the threshold.
        return Math
            .abs(rightShooter.getVelocity().getValueAsDouble()) >= Constants.ShooterConstants.shooterSpeedThresholdRPS;
      case BOTH:
        // Check if both shooters are at or above the speed threshold.
        return isAtSpeed(ShooterSide.LEFT) && isAtSpeed(ShooterSide.RIGHT);
      default:
        return false;
    }
  }

  /**
   * Checks if both shooters have reached their target speed.
   * @return True if both shooters are at or above the speed threshold, false otherwise.
   */
  public boolean areShootersAtSpeed() {
    return isAtSpeed(ShooterSide.BOTH);
  }

  /**
   * Checks if the left shooter has reached its target speed.
   * @return True if the left shooter is at or above the speed threshold, false otherwise.
   */
  public boolean isLeftAtSpeed() {
    return isAtSpeed(ShooterSide.LEFT);
  }

  /**
   * Checks if the right shooter has reached its target speed.
   * @return True if the right shooter is at or above the speed threshold, false otherwise.
   */
  public boolean isRightAtSpeed() {
    return isAtSpeed(ShooterSide.RIGHT);
  }

  /**
   * Retrieves the current distance to the hub.
   * @return The distance to the hub in meters.
   */
  public double getHubDistance() {
    return AllianceUtil.getDistanceToHub(drivetrain, limelightName);
  }

  /**
   * Returns a command that runs the left shooter to its target speed.
   * The command finishes when the left shooter reaches the speed threshold.
   * @return A command to run the left shooter to speed.
   */
  public Command runLeftShooterToSpeedCommand() {
    return run(() -> {
      double target = (controlMode == ShooterControlMode.VELOCITY)
          ? Constants.ShooterConstants.shooterVelocityRPS // Target velocity in RPS
          : Constants.ShooterConstants.shooterTargetVoltage; // Target voltage
      // Set the output for the left shooter, keeping the right shooter at 0.
      setShooterOutput(target, 0);
    }).until(
        () -> isAtSpeed(ShooterSide.LEFT)).withName("RunLeftShooterToSpeed");
  }

  /**
   * Returns a command that runs the right shooter to its target speed.
   * The command finishes when the right shooter reaches the speed threshold.
   * Note: Right shooter target is negative to spin in the opposite direction.
   * @return A command to run the right shooter to speed.
   */
  public Command runRightShooterToSpeedCommand() {
    return run(() -> {
      double target = (controlMode == ShooterControlMode.VELOCITY)
          ? -Constants.ShooterConstants.shooterVelocityRPS // Target velocity in RPS (negative for right shooter)
          : -Constants.ShooterConstants.shooterTargetVoltage; // Target voltage (negative for right shooter)
      // Set the output for the right shooter, keeping the left shooter at 0.
      setShooterOutput(0, target);
    }).until(
        () -> isAtSpeed(ShooterSide.RIGHT)).withName("RunRightShooterToSpeed");
  }

  /**
   * Returns a command that runs both shooters to their target speed.
   * The command finishes when both shooters reach the speed threshold.
   * @return A command to run both shooters to speed.
   */
  public Command runBothShootersToSpeedCommand() {
    return run(
        () -> {
          // Determine the target output based on the current control mode.
          double leftTarget = (controlMode == ShooterControlMode.VELOCITY)
              ? Constants.ShooterConstants.shooterVelocityRPS // Target velocity in RPS
              : Constants.ShooterConstants.shooterTargetVoltage; // Target voltage

          // Right shooter spins in the opposite direction, so its target is negative.
          double rightTarget = -leftTarget;
          setShooterOutput(leftTarget, rightTarget);
        })
        .until(
            this::areShootersAtSpeed) // Command finishes when both shooters are at speed.
        .withName("RunBothShootersToSpeed");
  }

  /**
   * Stops the left shooter motor, setting its output to zero based on the current control mode.
   */
  public void stopLeftShooter() {
    if (controlMode == ShooterControlMode.VELOCITY) {
      leftShooter.setControl(velocityCtrl.withVelocity(0));
    } else {
      leftShooter.setControl(voltageCtrl.withOutput(0));
    }
  }

  /**
   * Stops the right shooter motor, setting its output to zero based on the current control mode.
   */
  public void stopRightShooter() {
    if (controlMode == ShooterControlMode.VELOCITY) {
      rightShooter.setControl(velocityCtrl.withVelocity(0));
    } else {
      rightShooter.setControl(voltageCtrl.withOutput(0));
    }
  }

  /**
   * Stops both shooter motors.
   */
  public void stopShooters() {
    stopLeftShooter();
    stopRightShooter();
  }

  /**
   * Returns a command that stops the left shooter.
   * @return A command to stop the left shooter.
   */
  public Command stopLeftShooterCommand() {
    return runOnce(this::stopLeftShooter);
  }

  /**
   * Returns a command that stops the right shooter.
   * @return A command to stop the right shooter.
   */
  public Command stopRightShooterCommand() {
    return runOnce(this::stopRightShooter);
  }

  /**
   * Returns a command that stops both shooters.
   * @return A command to stop both shooters.
   */
  public Command stopShootersCommand() {
    return runOnce(this::stopShooters);
  }

  /**
   * Sets the target output for both shooters based on a given distance to the target.
   * Uses interpolation tables defined in Constants to convert distance to target velocity or voltage.
   * @param distance The distance to the target in meters.
   */
  public void setTargetFromDistance(double distance) {
    if (controlMode == ShooterControlMode.VELOCITY) {
      // Get target RPS from distance using interpolation table.
      double rps = Constants.ShooterConstants.distanceToVelocityRPS.get(distance);
      setShooterOutput(rps, -rps); // Right shooter target is negative.
    } else {
      // Get target voltage from distance using interpolation table.
      double volts = Constants.ShooterConstants.distanceToVoltage.get(distance);
      setShooterOutput(volts, -volts); // Right shooter target is negative.
    }
  }

  /**
   * Returns a command that runs the shooters at a speed interpolated from the distance to the target.
   * The command will stop the shooters when it ends.
   * @param distanceMeters A DoubleSupplier providing the current distance to the target.
   * @return A command to run interpolated shot.
   */
  public Command runInterpolatedShot(DoubleSupplier distanceMeters) {
    return run(
        () -> {
          double d = distanceMeters.getAsDouble();
          setTargetFromDistance(d);
        })
        .finallyDo(this::stopShooters); // Ensure shooters stop when the command ends.
  }

  /**
   * Sets the target Revolutions Per Second (RPS) for both shooters.
   * Only applicable when in VELOCITY control mode.
   * @param rps The target RPS for the left shooter. The right shooter will be set to -rps.
   */
  public void setTargetRPS(double rps) {
    setShooterOutput(rps, -rps);
  }

  /**
   * Sets the output for the left and right shooter motors.
   * The interpretation of `leftValue` and `rightValue` depends on the current `controlMode`.
   * If in VELOCITY mode, values are RPS. If in VOLTAGE mode, values are Volts.
   * @param leftValue The target value for the left shooter.
   * @param rightValue The target value for the right shooter.
   */
  private void setShooterOutput(double leftValue, double rightValue) {
    if (controlMode == ShooterControlMode.VELOCITY) {
      // Store setpoints for telemetry.
      lastLeftSetpointRps = leftValue;
      lastRightSetpointRps = rightValue;
      // Apply velocity control.
      leftShooter.setControl(velocityCtrl.withVelocity(leftValue));
      rightShooter.setControl(velocityCtrl.withVelocity(rightValue));
    } else {
      // Apply voltage control.
      leftShooter.setControl(voltageCtrl.withOutput(leftValue));
      rightShooter.setControl(voltageCtrl.withOutput(rightValue));
    }
  }

  /**
   * Called periodically by the scheduler.
   * Updates SmartDashboard with shooter status, current speeds, and setpoints.
   */
  @Override
  public void periodic() {
    SmartDashboard.putString("Shooter/Mode", controlMode.toString());
    SmartDashboard.putNumber("Shooter/LeftRPS", leftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/RightRPS", rightShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter/LeftAtSpeed", isAtSpeed(ShooterSide.LEFT));
    SmartDashboard.putBoolean("Shooter/RightAtSpeed", isAtSpeed(ShooterSide.RIGHT));
    SmartDashboard.putNumber("Shooter/LeftSetpointRPS", lastLeftSetpointRps);
    SmartDashboard.putNumber("Shooter/RightSetpointRPS", lastRightSetpointRps);
    // Log the distance to the hub for calibration and verification.
    SmartDashboard.putNumber("Shooter/DistanceMeters", getHubDistance());
  }
}
