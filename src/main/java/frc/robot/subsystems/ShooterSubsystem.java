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

  private final TalonFX leftShooter = new TalonFX(Constants.ShooterConstants.leftShooterId);
  private final TalonFX rightShooter = new TalonFX(Constants.ShooterConstants.rightShooterId);

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

    leftShooter.getConfigurator().apply(cfg);
    rightShooter.getConfigurator().apply(cfg);

    leftShooter.setNeutralMode(NeutralModeValue.Coast);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setControlMode(ShooterControlMode mode) {
    this.controlMode = mode;
  }

  public boolean isAtSpeed(ShooterSide side) {
    switch (side) {
      case LEFT:
        return Math
            .abs(leftShooter.getVelocity().getValueAsDouble()) >= Constants.ShooterConstants.shooterSpeedThresholdRPS;
      case RIGHT:
        return Math
            .abs(rightShooter.getVelocity().getValueAsDouble()) >= Constants.ShooterConstants.shooterSpeedThresholdRPS;
      default:
        return false;
    }
  }

  public boolean areShootersAtSpeed() {
    return isAtSpeed(ShooterSide.BOTH);
  }

  public Command runLeftShooterToSpeedCommand() {
    return run(() -> {
      double target = (controlMode == ShooterControlMode.VELOCITY)
          ? Constants.ShooterConstants.shooterVelocityRPS
          : Constants.ShooterConstants.shooterTargetVoltage;
      setShooterOutput(target, 0);
    }).until(
        () -> isAtSpeed(ShooterSide.LEFT)).withName("RunLeftShooterToSpeed");
  }

  public Command runRightShooterToSpeedCommand() {
    return run(() -> {
      double target = (controlMode == ShooterControlMode.VELOCITY)
          ? -Constants.ShooterConstants.shooterVelocityRPS
          : -Constants.ShooterConstants.shooterTargetVoltage;
      setShooterOutput(0, target);
    }).until(
        () -> isAtSpeed(ShooterSide.RIGHT)).withName("RunRightShooterToSpeed");
  }

  public Command runBothShootersToSpeedCommand() {
    return run(
        () -> {
          double leftTarget = (controlMode == ShooterControlMode.VELOCITY)
              ? Constants.ShooterConstants.shooterVelocityRPS
              : Constants.ShooterConstants.shooterTargetVoltage;

          double rightTarget = -leftTarget;
          setShooterOutput(leftTarget, rightTarget);
        })
        .until(
            this::areShootersAtSpeed)
        .withName("RunBothShootersToSpeed");
  }

  public void stopLeftShooter() {
    if (controlMode == ShooterControlMode.VELOCITY) {
      leftShooter.setControl(velocityCtrl.withVelocity(0));
    } else {
      leftShooter.setControl(voltageCtrl.withOutput(0));
    }
  }

  public void stopRightShooter() {
    if (controlMode == ShooterControlMode.VELOCITY) {
      rightShooter.setControl(velocityCtrl.withVelocity(0));
    } else {
      rightShooter.setControl(voltageCtrl.withOutput(0));
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
      lastLeftSetpointRps = leftValue;
      leftShooter.setControl(velocityCtrl.withVelocity(leftValue));
      lastRightSetpointRps = rightValue;
      rightShooter.setControl(velocityCtrl.withVelocity(rightValue));
    } else {
      leftShooter.setControl(voltageCtrl.withOutput(leftValue));
      rightShooter.setControl(voltageCtrl.withOutput(rightValue));
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Shooter/Mode", controlMode.toString());
    SmartDashboard.putNumber("Shooter/LeftRPS", leftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/RightRPS", rightShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter/LeftAtSpeed", isAtSpeed(ShooterSide.LEFT));
    SmartDashboard.putBoolean("Shooter/RightAtSpeed", isAtSpeed(ShooterSide.RIGHT));
    SmartDashboard.putNumber("Shooter/LeftSetpointRPS", lastLeftSetpointRps);
    SmartDashboard.putNumber("Shooter/RightSetpointRPS", lastRightSetpointRps);
  }
}
