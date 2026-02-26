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

  private final TalonFX leftShooter = new TalonFX(Constants.ShooterConstants.leftShooterId);
  private final TalonFX rightShooter = new TalonFX(Constants.ShooterConstants.rightShooterId);

  private final VelocityVoltage velocityCtrl = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageCtrl = new VoltageOut(0);

  /**
   * Represents a setpoint for the shooter, encapsulating both revolutions per second (RPS) and voltage.
   */
  private static class ShooterSetpoint {
    public final double rps;
    public final double voltage;

    public ShooterSetpoint(double rps, double voltage) {
      this.rps = rps;
      this.voltage = voltage;
    }
  }

  private ShooterSetpoint lastLeftSetpoint = new ShooterSetpoint(0.0, 0.0);
  private ShooterSetpoint lastRightSetpoint = new ShooterSetpoint(0.0, 0.0);

  private double rpsAdjustment = 0.0;

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

  private ShooterControlMode controlMode = ShooterControlMode.VELOCITY;

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
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0 = new Slot0Configs()
        .withKP(Constants.ShooterConstants.kP)
        .withKI(Constants.ShooterConstants.kI)
        .withKD(Constants.ShooterConstants.kD)
        .withKV(Constants.ShooterConstants.kV);

    cfg.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.shooterCurrentLimit;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.shooterCurrentLimit;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    leftShooter.getConfigurator().apply(cfg);
    rightShooter.getConfigurator().apply(cfg);

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

  public void incrementRPSAdjustment(double delta) {
    this.rpsAdjustment += delta;
  }

  public void resetRPSAdjustment() {
    this.rpsAdjustment = 0.0;
  }

  /**
   * Checks if a specific shooter side (or both) has reached its target speed.
   * @param side The shooter side to check (LEFT, RIGHT).
   * @return True if the specified shooter(s) are at or above the speed threshold, false otherwise.
   */
  public boolean isAtSpeed(ShooterSide side) {
    double currentLeftRPS = leftShooter.getVelocity().getValueAsDouble();
    double currentRightRPS = rightShooter.getVelocity().getValueAsDouble();


    double targetLeftRPS = (controlMode == ShooterControlMode.VELOCITY)
        ? lastLeftSetpoint.rps
        : lastLeftSetpoint.voltage / Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;
    double targetRightRPS = (controlMode == ShooterControlMode.VELOCITY)
        ? lastRightSetpoint.rps
        : lastRightSetpoint.voltage / Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;

    switch (side) {
      case LEFT:
        return Math.abs(currentLeftRPS - targetLeftRPS) <= Constants.ShooterConstants.shooterSpeedToleranceRPS;
      case RIGHT:
        return Math.abs(currentRightRPS - targetRightRPS) <= Constants.ShooterConstants.shooterSpeedToleranceRPS;
      case BOTH:
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
   * Sets the target output for both shooters based on a given distance to the target.
   * Uses interpolation tables defined in Constants to convert distance to target velocity or voltage.
   * @param distance The distance to the target in meters.
   */
  public void setTargetFromDistance(double distance) {
    double rps = Constants.ShooterConstants.distanceToVelocityRPS.get(distance);
    double adjustedRPS = rps + rpsAdjustment;
    double volts = adjustedRPS / Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT; // Recalculate voltage based on adjusted RPS

    ShooterSetpoint leftSetpoint = new ShooterSetpoint(adjustedRPS, volts);
    ShooterSetpoint rightSetpoint = new ShooterSetpoint(-adjustedRPS, -volts);
    setShooterOutput(leftSetpoint, rightSetpoint);
  }

  /**
   * Sets the target Revolutions Per Second (RPS) for both shooters.
   * Only applicable when in VELOCITY control mode.
   * @param rps The target RPS for the left shooter. The right shooter will be set to -rps.
   */
  public void setTargetRPS(double rps) {
    double adjustedRPS = rps + rpsAdjustment;
    double voltage = adjustedRPS / Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;
    ShooterSetpoint leftSetpoint = new ShooterSetpoint(adjustedRPS, voltage);
    ShooterSetpoint rightSetpoint = new ShooterSetpoint(-adjustedRPS, -voltage);
    setShooterOutput(leftSetpoint, rightSetpoint);
  }

  /**
   * Sets the output for the left and right shooter motors.
   * The interpretation of `leftValue` and `rightValue` depends on the current `controlMode`.
   * If in VELOCITY mode, values are RPS. If in VOLTAGE mode, values are Volts.
   * @param leftValue The target value for the left shooter.
   * @param rightValue The target value for the right shooter.
   */
  private void setShooterOutput(ShooterSetpoint leftSetpoint, ShooterSetpoint rightSetpoint) {
    if (controlMode == ShooterControlMode.VELOCITY) {
      lastLeftSetpoint = leftSetpoint;
      lastRightSetpoint = rightSetpoint;
      leftShooter.setControl(velocityCtrl.withVelocity(leftSetpoint.rps));
      rightShooter.setControl(velocityCtrl.withVelocity(rightSetpoint.rps));
    } else {
      lastLeftSetpoint = leftSetpoint;
      lastRightSetpoint = rightSetpoint;
      leftShooter.setControl(voltageCtrl.withOutput(leftSetpoint.voltage));
      rightShooter.setControl(voltageCtrl.withOutput(rightSetpoint.voltage));
    }
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
   * Returns a command that runs the left shooter to its target speed.
   * The command finishes when the left shooter reaches the speed threshold.
   * @return A command to run the left shooter to speed.
   */
  public Command runLeftShooterToSpeedCommand() {
    return run(() -> {
      ShooterSetpoint leftSetpoint;
      ShooterSetpoint rightSetpoint = new ShooterSetpoint(0.0, 0.0);

      if (controlMode == ShooterControlMode.VELOCITY) {
        double targetRPS = Constants.ShooterConstants.shooterVelocityRPS;
        double targetVoltage = targetRPS / Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;
        leftSetpoint = new ShooterSetpoint(targetRPS, targetVoltage);
      } else { // VOLTAGE mode
        double targetVoltage = Constants.ShooterConstants.shooterTargetVoltage;
        double targetRPS = targetVoltage * Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;
        leftSetpoint = new ShooterSetpoint(targetRPS, targetVoltage);
      }
      setShooterOutput(leftSetpoint, rightSetpoint);
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
      ShooterSetpoint leftSetpoint = new ShooterSetpoint(0.0, 0.0);
      ShooterSetpoint rightSetpoint;

      if (controlMode == ShooterControlMode.VELOCITY) {
        double targetRPS = -Constants.ShooterConstants.shooterVelocityRPS;
        double targetVoltage = targetRPS / Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;
        rightSetpoint = new ShooterSetpoint(targetRPS, targetVoltage);
      } else { // VOLTAGE mode
        double targetVoltage = -Constants.ShooterConstants.shooterTargetVoltage;
        double targetRPS = targetVoltage * Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;
        rightSetpoint = new ShooterSetpoint(targetRPS, targetVoltage);
      }
      setShooterOutput(leftSetpoint, rightSetpoint);
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
          ShooterSetpoint leftSetpoint;
          ShooterSetpoint rightSetpoint;

          if (controlMode == ShooterControlMode.VELOCITY) {
            double targetRPS = Constants.ShooterConstants.shooterVelocityRPS;
            double targetVoltage = targetRPS / Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;
            leftSetpoint = new ShooterSetpoint(targetRPS, targetVoltage);
            rightSetpoint = new ShooterSetpoint(-targetRPS, -targetVoltage);
          } else { // VOLTAGE mode
            double targetVoltage = Constants.ShooterConstants.shooterTargetVoltage;
            double targetRPS = targetVoltage * Constants.ShooterConstants.SHOOTER_KV_RPS_PER_VOLT;
            leftSetpoint = new ShooterSetpoint(targetRPS, targetVoltage);
            rightSetpoint = new ShooterSetpoint(-targetRPS, -targetVoltage);
          }
          setShooterOutput(leftSetpoint, rightSetpoint);
        })
        .until(this::areShootersAtSpeed)
        .withName("RunBothShootersToSpeed");
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
        .until(this::areShootersAtSpeed)
        .finallyDo(this::stopShooters);
  }

  /**
   * Returns a command that runs the shooters at a fixed RPS.
   * The command will stop the shooters when it ends.
   * @param rps The target Revolutions Per Second for the shooters.
   * @return A command to run shooters at a fixed RPS.
   */
  public Command runFixedRPSShoot(double rps) {
    return run(
        () -> setTargetRPS(rps))
        .until(this::areShootersAtSpeed)
        .finallyDo(this::stopShooters);
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
    SmartDashboard.putNumber("Shooter/LeftSetpointRPS", lastLeftSetpoint.rps);
    SmartDashboard.putNumber("Shooter/LeftSetpointVoltage", lastLeftSetpoint.voltage);
    SmartDashboard.putNumber("Shooter/RightSetpointRPS", lastRightSetpoint.rps);
    SmartDashboard.putNumber("Shooter/RightSetpointVoltage", lastRightSetpoint.voltage);
    SmartDashboard.putNumber("Shooter/DistanceMeters", getHubDistance());
  }
}
