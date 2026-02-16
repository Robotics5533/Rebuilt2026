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
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX leftMaster = new TalonFX(Constants.ShooterConstants.leftShooterMasterId);
  private final TalonFX leftFollower = new TalonFX(Constants.ShooterConstants.leftShooterFollowerId);
  private final TalonFX rightMaster = new TalonFX(Constants.ShooterConstants.rightShooterMasterId);
  private final TalonFX rightFollower = new TalonFX(Constants.ShooterConstants.rightShooterFollowerId);

  private final VelocityVoltage velocityCtrl = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageCtrl = new VoltageOut(0);

  private double lastLeftSetpointRps = 0.0;
  private double lastRightSetpointRps = 0.0;

  public enum ShooterSide {
    LEFT,
    RIGHT,
    BOTH
  }

  public enum ShooterControlMode {
    VOLTAGE,
    VELOCITY
  }

  private ShooterControlMode controlMode = ShooterControlMode.VELOCITY;

  public ShooterSubsystem() {
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

    leftMaster.getConfigurator().apply(cfg);
    leftFollower.getConfigurator().apply(cfg);
    rightMaster.getConfigurator().apply(cfg);
    rightFollower.getConfigurator().apply(cfg);

    leftMaster.setNeutralMode(NeutralModeValue.Coast);
    leftFollower.setNeutralMode(NeutralModeValue.Coast);
    rightMaster.setNeutralMode(NeutralModeValue.Coast);
    rightFollower.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setControlMode(ShooterControlMode mode) {
    this.controlMode = mode;
  }

  public boolean isAtSpeed(ShooterSide side) {
    switch (side) {
      case LEFT:
        return Math
            .abs(leftMaster.getVelocity().getValueAsDouble()) >= Constants.ShooterConstants.shooterSpeedThresholdRPS;
      case RIGHT:
        return Math
            .abs(rightMaster.getVelocity().getValueAsDouble()) >= Constants.ShooterConstants.shooterSpeedThresholdRPS;
      case BOTH:
        return isAtSpeed(ShooterSide.LEFT) && isAtSpeed(ShooterSide.RIGHT);
      default:
        return false;
    }
  }

  public boolean areShootersAtSpeed() {
    return isAtSpeed(ShooterSide.BOTH);
  }

  public Command runLeftShooter() {
    return run(
        () -> {
          if (controlMode == ShooterControlMode.VELOCITY) {
            setLeftVelocity(Constants.ShooterConstants.shooterVelocityRPS);
          } else {
            setLeftVoltage(Constants.ShooterConstants.shooterTargetVoltage);
          }
        })
        .finallyDo(this::stopLeftShooter);
  }

  public Command runRightShooter() {
    return run(
        () -> {
          if (controlMode == ShooterControlMode.VELOCITY) {
            setRightVelocity(-Constants.ShooterConstants.shooterVelocityRPS);
          } else {
            setRightVoltage(-Constants.ShooterConstants.shooterTargetVoltage);
          }
        })
        .finallyDo(this::stopRightShooter);
  }

  public Command runBothShooters() {
    return run(
        () -> {
          double leftTarget = (controlMode == ShooterControlMode.VELOCITY)
              ? Constants.ShooterConstants.shooterVelocityRPS
              : Constants.ShooterConstants.shooterTargetVoltage;

          double rightTarget = -leftTarget;
          setShooterOutput(leftTarget, rightTarget);
        })
        .finallyDo(this::stopShooters);
  }

  public void startBothShooters() {
    double leftTarget = (controlMode == ShooterControlMode.VELOCITY)
        ? Constants.ShooterConstants.shooterVelocityRPS
        : Constants.ShooterConstants.shooterTargetVoltage;

    double rightTarget = -leftTarget;
    setShooterOutput(leftTarget, rightTarget);
  }

  public void stopLeftShooter() {
    if (controlMode == ShooterControlMode.VELOCITY) {
      setLeftVelocity(0);
    } else {
      setLeftVoltage(0);
    }
  }

  public void stopRightShooter() {
    if (controlMode == ShooterControlMode.VELOCITY) {
      setRightVelocity(0);
    } else {
      setRightVoltage(0);
    }
  }

  public void stopShooters() {
    stopLeftShooter();
    stopRightShooter();
  }

  public Command stopLeftShooterCommand() {
    return runOnce(this::stopLeftShooter);
  }

  public Command stopRightShooterCommand() {
    return runOnce(this::stopRightShooter);
  }

  public Command stopShootersCommand() {
    return runOnce(this::stopShooters);
  }

  public void setTargetFromDistance(double distance) {
    if (controlMode == ShooterControlMode.VELOCITY) {
      double rps = Constants.ShooterConstants.distanceToVelocityRPS.get(distance);
      setShooterOutput(rps, -rps);
    } else {
      double volts = Constants.ShooterConstants.distanceToVoltage.get(distance);
      setShooterOutput(volts, -volts);
    }
  }

  public Command runInterpolatedShot(DoubleSupplier distanceMeters) {
    return run(
        () -> {
          double d = distanceMeters.getAsDouble();
          setTargetFromDistance(d);
        })
        .finallyDo(this::stopShooters);
  }

  public void setTargetRPS(double rps) {
    setShooterOutput(rps, -rps);
  }

  private void setShooterOutput(double leftValue, double rightValue) {
    if (controlMode == ShooterControlMode.VELOCITY) {
      setLeftVelocity(leftValue);
      setRightVelocity(rightValue);
    } else {
      setLeftVoltage(leftValue);
      setRightVoltage(rightValue);
    }
  }

  private void setLeftVoltage(double volts) {
    leftMaster.setControl(voltageCtrl.withOutput(volts));
    leftFollower.setControl(voltageCtrl.withOutput(volts));
  }

  private void setRightVoltage(double volts) {
    rightMaster.setControl(voltageCtrl.withOutput(volts));
    rightFollower.setControl(voltageCtrl.withOutput(volts));
  }

  private void setLeftVelocity(double rps) {
    lastLeftSetpointRps = rps;
    leftMaster.setControl(velocityCtrl.withVelocity(rps));
    leftFollower.setControl(velocityCtrl.withVelocity(rps));
  }

  private void setRightVelocity(double rps) {
    lastRightSetpointRps = rps;
    rightMaster.setControl(velocityCtrl.withVelocity(rps));
    rightFollower.setControl(velocityCtrl.withVelocity(rps));
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Shooter/Mode", controlMode.toString());
    SmartDashboard.putNumber("Shooter/LeftRPS", leftMaster.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/RightRPS", rightMaster.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter/LeftAtSpeed", isAtSpeed(ShooterSide.LEFT));
    SmartDashboard.putBoolean("Shooter/RightAtSpeed", isAtSpeed(ShooterSide.RIGHT));
    SmartDashboard.putNumber("Shooter/LeftSetpointRPS", lastLeftSetpointRps);
    SmartDashboard.putNumber("Shooter/RightSetpointRPS", lastRightSetpointRps);
  }
}
