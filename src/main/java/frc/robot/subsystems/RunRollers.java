package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.epilogue.Logged;
//import edu.wpi.first.epilogue.Epilogue;

//@Logged
public class RunRollers extends SubsystemBase {

  // Leader motor (runs PID/velocity control)
  private final TalonFX rollerMotorLeft = new TalonFX(Constants.IntakeConstants.intakeRollerMotorLeftId);
  
  // Follower motor (mirrors the leader's output)
  private final TalonFX rollerMotorRight = new TalonFX(Constants.IntakeConstants.intakeRollerMotorRightId);

  private final VelocityVoltage velocityCtrl = new VelocityVoltage(0).withSlot(0);
  private final Follower followerCtrl = new Follower(Constants.IntakeConstants.intakeRollerMotorLeftId, true);

  /**
   * Represents a setpoint for the rollers, encapsulating both revolutions per second (RPS) and voltage.
   */
  private static class RollerSetpoint {
    public final double rps;
    public final double voltage;

    public RollerSetpoint(double rps, double voltage) {
      this.rps = rps;
      this.voltage = voltage;
    }
  }

  private RollerSetpoint lastSetpoint = new RollerSetpoint(0.0, 0.0);

  public RunRollers() {
    // Configure leader motor (left) with PID and feedforward
    TalonFXConfiguration leaderCfg = new TalonFXConfiguration();

    leaderCfg.Slot0 = new Slot0Configs()
        .withKP(Constants.IntakeConstants.rollerKP)
        .withKI(Constants.IntakeConstants.rollerKI)
        .withKD(Constants.IntakeConstants.rollerKD)
        .withKV(Constants.IntakeConstants.rollerKV);

    leaderCfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.rollerCurrentLimit;
    leaderCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderCfg.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.rollerCurrentLimit;
    leaderCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerMotorLeft.getConfigurator().apply(leaderCfg);
    rollerMotorLeft.setNeutralMode(NeutralModeValue.Coast);

    // Configure follower motor (right) with same current limits
    TalonFXConfiguration followerCfg = new TalonFXConfiguration();
    followerCfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.rollerCurrentLimit;
    followerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerCfg.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.rollerCurrentLimit;
    followerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerMotorRight.getConfigurator().apply(followerCfg);
    rollerMotorRight.setNeutralMode(NeutralModeValue.Coast);

    // Set right motor to follow left motor (with opposite direction for gear arrangement)
    rollerMotorRight.setControl(followerCtrl);
  }
  

  /* ----- Speed Checking Methods ----- */

  /**
   * Checks if the roller leader motor has reached its target speed.
   * Since the follower mirrors the leader, checking the leader is sufficient.
   * @return True if the leader is at target speed, false otherwise.
   */
  public boolean isAtSpeed() {
    double currentRPS = rollerMotorLeft.getVelocity().getValueAsDouble();
    double targetRPS = lastSetpoint.rps;
    return Math.abs(currentRPS - targetRPS) <= Constants.IntakeConstants.rollerSpeedToleranceRPS;
  }

  /**
   * Gets the current velocity of the leader motor in RPS.
   * @return Current RPS of the roller motors.
   */
  public double getCurrentRPS() {
    return rollerMotorLeft.getVelocity().getValueAsDouble();
  }

  /**
   * Gets the target velocity setpoint in RPS.
   * @return Target RPS.
   */
  public double getTargetRPS() {
    return lastSetpoint.rps;
  }

  /* ----- Roller Control Methods ----- */

  /**
   * Sets the target Revolutions Per Second (RPS) for the roller motors.
   * The left motor (leader) runs with PID velocity control, and the right motor (follower) mirrors its output.
   * @param rps The target RPS for the rollers.
   */
  public void setTargetRPS(double rps) {
    double voltage = rps / Constants.IntakeConstants.ROLLER_KV_RPS_PER_VOLT;
    lastSetpoint = new RollerSetpoint(rps, voltage);
    rollerMotorLeft.setControl(velocityCtrl.withVelocity(rps));
  }

  /**
   * Runs the rollers forward at the target velocity.
   */
  public void runRollerForward() {
    setTargetRPS(Constants.IntakeConstants.rollerTargetVelocityRPS);
  }

  /**
   * Runs the rollers in reverse at the target velocity.
   */
  public void runRollerReverse() {
    setTargetRPS(-Constants.IntakeConstants.rollerTargetVelocityRPS);
  }

  /**
   * Stops the rollers.
   */
  public void stopRoller() {
    setTargetRPS(0.0);
  }

  /* ----- Command Methods ----- */

  /**
   * Returns a command that runs the rollers forward to their target speed.
   * The command finishes when the rollers reach the speed threshold.
   * @return A command to run the rollers forward to speed.
   */
  public Command runRollerForwardCommand() {
    return run(this::runRollerForward)
        .until(this::isAtSpeed)
        .withName("RunRollerForward");
  }

  /**
   * Returns a command that runs the rollers in reverse to their target speed.
   * The command finishes when the rollers reach the speed threshold.
   * @return A command to run the rollers in reverse to speed.
   */
  public Command runRollerReverseCommand() {
    return run(this::runRollerReverse)
        .until(this::isAtSpeed)
        .withName("RunRollerReverse");
  }

  /**
   * Returns a command that stops the rollers.
   * @return A command to stop the rollers.
   */
  public Command stopRollerCommand() {
    return runOnce(this::stopRoller)
        .withName("StopRoller");
  }

  /**
   * Returns a command that runs the rollers forward and keeps them running.
   * Does not finish until interrupted.
   * @return A command to indefinitely run the rollers forward.
   */
  public Command runRollerForwardIndefiniteCommand() {
    return run(this::runRollerForward)
        .withName("RunRollerForwardIndefinite");
  }

  /**
   * Returns a command that runs the rollers in reverse and keeps them running.
   * Does not finish until interrupted.
   * @return A command to indefinitely run the rollers in reverse.
   */
  public Command runRollerReverseIndefiniteCommand() {
    return run(this::runRollerReverse)
        .withName("RunRollerReverseIndefinite");
  }

  /**
   * Called periodically by the scheduler.
   * Updates SmartDashboard with roller status and current speeds.
   */
  @Override
  public void periodic() {
    // Both motors are synchronized via leader-follower, so we only need to monitor the leader
    SmartDashboard.putNumber("Roller/LeaderRPS", rollerMotorLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Roller/FollowerRPS", rollerMotorRight.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Roller/AtSpeed", isAtSpeed());
    SmartDashboard.putNumber("Roller/TargetRPS", lastSetpoint.rps);
    SmartDashboard.putNumber("Roller/TargetVoltage", lastSetpoint.voltage);
    SmartDashboard.putNumber("Roller/LeaderVelocityError", getCurrentRPS() - getTargetRPS());
  }
}