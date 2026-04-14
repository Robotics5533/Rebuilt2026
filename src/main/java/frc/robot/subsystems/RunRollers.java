package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RunRollers extends SubsystemBase {

  private final TalonFX rollerMotorLeft  = new TalonFX(Constants.IntakeConstants.intakeRollerMotorLeftId);
  private final TalonFX rollerMotorRight = new TalonFX(Constants.IntakeConstants.intakeRollerMotorRightId);

  private final VelocityVoltage velocityCtrl = new VelocityVoltage(0).withSlot(0);

  private double targetRPS = 0.0;

  public RunRollers() {
    // --- Leader (left) ---
    TalonFXConfiguration leaderCfg = new TalonFXConfiguration();
    leaderCfg.Slot0 = new Slot0Configs()
        .withKP(Constants.IntakeConstants.rollerKP)
        .withKI(Constants.IntakeConstants.rollerKI)
        .withKD(Constants.IntakeConstants.rollerKD)
        .withKV(Constants.IntakeConstants.rollerKV);
    leaderCfg.CurrentLimits.SupplyCurrentLimit       = Constants.IntakeConstants.rollerCurrentLimit;
    leaderCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderCfg.CurrentLimits.StatorCurrentLimit       = Constants.IntakeConstants.rollerCurrentLimit;
    leaderCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorLeft.getConfigurator().apply(leaderCfg);
    rollerMotorLeft.setNeutralMode(NeutralModeValue.Coast);

    // --- Follower (right) ---
    TalonFXConfiguration followerCfg = new TalonFXConfiguration();
    followerCfg.CurrentLimits.SupplyCurrentLimit       = Constants.IntakeConstants.rollerCurrentLimit;
    followerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerCfg.CurrentLimits.StatorCurrentLimit       = Constants.IntakeConstants.rollerCurrentLimit;
    followerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorRight.getConfigurator().apply(followerCfg);
    rollerMotorRight.setNeutralMode(NeutralModeValue.Coast);

    // Right motor follows left, opposed because they face opposite directions
    rollerMotorRight.setControl(
        new Follower(rollerMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed)
    );
  }

  /* ----- Getters ----- */

  public double getCurrentRPS() {
    return rollerMotorLeft.getVelocity().getValueAsDouble();
  }

  public double getTargetRPS() {
    return targetRPS;
  }

  public boolean isAtSpeed() {
    return Math.abs(getCurrentRPS() - targetRPS) <= Constants.IntakeConstants.rollerSpeedToleranceRPS;
  }

  /* ----- Control ----- */

  public void setTargetRPS(double rps) {
    targetRPS = rps;
    rollerMotorLeft.setControl(velocityCtrl.withVelocity(rps));
  }

  public void runRollerForward() {
    setTargetRPS(-Constants.IntakeConstants.rollerTargetVelocityRPS);
  }

  public void runRollerReverse() {
    setTargetRPS(Constants.IntakeConstants.rollerTargetVelocityRPS);
  }

  public void stopRoller() {
    setTargetRPS(0.0);
  }

  /* ----- Commands ----- */

  public Command runRollerForwardCommand() {
    return run(this::runRollerForward)
        .until(this::isAtSpeed)
        .withName("RunRollerForward");
  }

  public Command runRollerReverseCommand() {
    return run(this::runRollerReverse)
        .until(this::isAtSpeed)
        .withName("RunRollerReverse");
  }

  public Command stopRollerCommand() {
    return runOnce(this::stopRoller)
        .withName("StopRoller");
  }

  public Command runRollerForwardIndefiniteCommand() {
    return run(this::runRollerForward)
        .withName("RunRollerForwardIndefinite");
  }

  public Command runRollerReverseIndefiniteCommand() {
    return run(this::runRollerReverse)
        .withName("RunRollerReverseIndefinite");
  }

  /* ----- Periodic ----- */

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roller/LeaderRPS",     getCurrentRPS());
    SmartDashboard.putNumber("Roller/FollowerRPS",   rollerMotorRight.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Roller/TargetRPS",     targetRPS);
    SmartDashboard.putNumber("Roller/VelocityError", getCurrentRPS() - targetRPS);
    SmartDashboard.putBoolean("Roller/AtSpeed",      isAtSpeed());
  }
}