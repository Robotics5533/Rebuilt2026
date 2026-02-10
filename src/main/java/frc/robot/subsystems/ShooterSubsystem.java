package frc.robot.subsystems;

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
    LEFT, RIGHT, BOTH
  }

  public enum ShooterControlMode {
    VOLTAGE, VELOCITY
  }

  private ShooterControlMode controlMode = ShooterControlMode.VOLTAGE;

  public ShooterSubsystem() {
    var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration()
        .withSlot0(new com.ctre.phoenix6.configs.Slot0Configs()
            .withKP(Constants.ShooterConstants.kP)
            .withKI(Constants.ShooterConstants.kI)
            .withKD(Constants.ShooterConstants.kD)
            .withKV(Constants.ShooterConstants.kV));

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

  public boolean isAtSpeed(ShooterSide side) {
    switch (side) {
      case LEFT:
        return Math.abs(leftMaster.getVelocity().getValueAsDouble()) >= Constants.ShooterConstants.shooterSpeedThresholdRPS;
      case RIGHT:
        return Math.abs(rightMaster.getVelocity().getValueAsDouble()) >= Constants.ShooterConstants.shooterSpeedThresholdRPS;
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
    return run(() -> setLeftVoltage(Constants.ShooterConstants.shooterTargetVoltage))
        .finallyDo(this::stopShooters);
  }

  public Command runRightShooter() {
    return run(() -> setRightVoltage(Constants.ShooterConstants.shooterTargetVoltage))
        .finallyDo(this::stopShooters);
  }

  public Command runBothShooters() {
    return run(() -> {
      setLeftVoltage(Constants.ShooterConstants.shooterTargetVoltage);
      setRightVoltage(Constants.ShooterConstants.shooterTargetVoltage);
    }).finallyDo(this::stopShooters);
  }

  public void startBothShooters() {
    setLeftVoltage(Constants.ShooterConstants.shooterTargetVoltage);
    setRightVoltage(Constants.ShooterConstants.shooterTargetVoltage);
  }

  public void stopShooters() {
    setLeftVoltage(0);
    setRightVoltage(0);
  }

  public void setControlMode(ShooterControlMode mode) {
    this.controlMode = mode;
  }

  public void setTargetFromDistance(double distance) {
    if (controlMode == ShooterControlMode.VELOCITY) {
      double rps = Constants.ShooterConstants.distanceToVelocityRPS.get(distance);
      setLeftVelocity(rps);
      setRightVelocity(rps);
    } else {
      double volts = Constants.ShooterConstants.distanceToVoltage.get(distance);
      setLeftVoltage(volts);
      setRightVoltage(volts);
    }
  }

  public Command runInterpolatedShot(DoubleSupplier distanceMeters) {
    return run(() -> {
      double d = distanceMeters.getAsDouble();
      setTargetFromDistance(d);
    }).finallyDo(this::stopShooters);
  }

  public void setTargetRPS(double rps) {
    setLeftVelocity(rps);
    setRightVelocity(rps);
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
    SmartDashboard.putNumber("Shooter/LeftRPS", leftMaster.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/RightRPS", rightMaster.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/LeftSetpointRPS", lastLeftSetpointRps);
    SmartDashboard.putNumber("Shooter/RightSetpointRPS", lastRightSetpointRps);
  }
}
