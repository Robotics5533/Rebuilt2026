package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  public enum State {
    Inactive, Active, Hanging
  }

  private final TalonFX motor = new TalonFX(Constants.ClimbConstants.climbMotorId);
  private final MotionMagicVoltage mmCtrl = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut climbVoltageCtrl = new VoltageOut(0);

  private State state = State.Inactive;
  private double targetInches = Constants.ClimbConstants.inactivePositionInches;
  private double climbZeroRot = 0.0;

  public ClimbSubsystem() {
    var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    cfg.Slot0.kP = Constants.ClimbConstants.kP;
    cfg.Slot0.kI = Constants.ClimbConstants.kI;
    cfg.Slot0.kD = Constants.ClimbConstants.kD;
    cfg.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    cfg.MotionMagic.MotionMagicAcceleration = 160.0;
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
    if (state == State.Inactive) {
      motor.setControl(climbVoltageCtrl.withOutput(0));
      return;
    }
    double targetRot = targetInches / Constants.ClimbConstants.rotationsToInches;
    motor.setControl(mmCtrl.withPosition(targetRot + climbZeroRot).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    applySetpoint();
    double rot = motor.getPosition().getValueAsDouble() - climbZeroRot;
    double inches = rot * Constants.ClimbConstants.rotationsToInches;
    double goalInches = targetInches;
    SmartDashboard.putNumber("Climb/Rot", rot);
    SmartDashboard.putNumber("Climb/Inches", inches);
    SmartDashboard.putNumber("Climb/GoalRot", targetInches / Constants.ClimbConstants.rotationsToInches);
    SmartDashboard.putNumber("Climb/GoalInches", goalInches);
  }

  public State getState() { return state; }

  private final SysIdRoutine sysIdClimb = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (volts) -> motor.setControl(climbVoltageCtrl.withOutput(volts)),
          (log) -> {
            log.motor("climb")
                .voltage(Units.Volts.of(motor.getMotorVoltage().getValueAsDouble()))
                .angularPosition(Units.Rotations.of(motor.getPosition().getValueAsDouble()))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getVelocity().getValueAsDouble()));
          },
          this));

  public edu.wpi.first.wpilibj2.command.Command sysIdQuasistaticForward() {
    return sysIdClimb.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdQuasistaticReverse() {
    return sysIdClimb.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdDynamicForward() {
    return sysIdClimb.dynamic(SysIdRoutine.Direction.kForward);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdDynamicReverse() {
    return sysIdClimb.dynamic(SysIdRoutine.Direction.kReverse);
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
