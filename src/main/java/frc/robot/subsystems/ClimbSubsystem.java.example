package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  public enum State {
    Inactive, Active, Hanging
  }

  private final TalonFX motor = new TalonFX(Constants.ClimbConstants.climbMotorId);
  private final MotionMagicVoltage mmCtrl = new MotionMagicVoltage(0).withSlot(0);

  private State state = State.Inactive;
  private double targetInches = Constants.ClimbConstants.inactivePositionInches;
  private double climbZeroRot = 0.0;

  public ClimbSubsystem() {
    var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    cfg.Feedback.SensorToMechanismRatio = Constants.ClimbConstants.climbGearRatio;

    cfg.Slot0.kP = Constants.ClimbConstants.kP;
    cfg.Slot0.kI = Constants.ClimbConstants.kI;
    cfg.Slot0.kD = Constants.ClimbConstants.kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimbConstants.climbCruiseVelocityRPS;
    cfg.MotionMagic.MotionMagicAcceleration = Constants.ClimbConstants.climbAccelerationRPS2;

    cfg.CurrentLimits.SupplyCurrentLimit = Constants.ClimbConstants.climbCurrentLimit;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = Constants.ClimbConstants.climbCurrentLimit;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimbConstants.softLimitForwardInches / Constants.ClimbConstants.rotationsToInches;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ClimbConstants.softLimitReverseInches / Constants.ClimbConstants.rotationsToInches;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    motor.getConfigurator().apply(cfg);
    motor.setNeutralMode(NeutralModeValue.Brake);
    climbZeroRot = motor.getPosition().getValueAsDouble();
  }

  public void setState(State s) {
    state = s;
    switch (s) {
      case Inactive -> targetInches = Constants.ClimbConstants.inactivePositionInches;
      case Active -> targetInches = Constants.ClimbConstants.activePositionInches;
      case Hanging -> targetInches = Constants.ClimbConstants.hangingPositionInches;
    }
  }

  public void cycleState() {
    switch (state) {
      case Inactive -> setState(State.Active);
      case Active -> setState(State.Hanging);
      case Hanging -> setState(State.Inactive);
    }
  }

  public void applySetpoint() {
        double targetRot = targetInches / Constants.ClimbConstants.rotationsToInches;
        motor.setControl(mmCtrl.withPosition(targetRot + climbZeroRot)
                .withFeedForward(Constants.ClimbConstants.kG)
                .withEnableFOC(true));
    }

  public double getPositionInches() {
    return (motor.getPosition().getValueAsDouble() - climbZeroRot) * Constants.ClimbConstants.rotationsToInches;
  }

  @Override
  public void periodic() {
    applySetpoint();
    double rot = motor.getPosition().getValueAsDouble() - climbZeroRot;
    double inches = rot * Constants.ClimbConstants.rotationsToInches;
    SmartDashboard.putNumber("Climb/Rot", rot);
    SmartDashboard.putNumber("Climb/Inches", inches);
    SmartDashboard.putNumber("Climb/TargetInches", targetInches);
    SmartDashboard.putNumber("Climb/ErrorInches", targetInches - inches);
  }

  public State getState() {
    return state;
  }

  public Command goInactive() {
    return runOnce(() -> setState(State.Inactive));
  }

  public Command goActive() {
    return runOnce(() -> setState(State.Active));
  }

  public Command goHangingDisableImmune() {
    return runOnce(() -> setState(State.Hanging)).ignoringDisable(true);
  }
}
