package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public enum FlipState {
    In, Out
  }

  private final TalonFX flipMotor = new TalonFX(Constants.IntakeConstants.intakeFlipMotorId);
  private final TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.intakeRollerMotorId);

  private final VoltageOut rollerCtrl = new VoltageOut(0);
  private final VoltageOut flipVoltageCtrl = new VoltageOut(0);
  private final ProfiledPIDController flipPid = new ProfiledPIDController(
      Constants.IntakeConstants.flipkP,
      Constants.IntakeConstants.flipkI,
      Constants.IntakeConstants.flipkD,
      new TrapezoidProfile.Constraints(
          Constants.IntakeConstants.flipMaxVelocityRotPerS,
          Constants.IntakeConstants.flipMaxAccelRotPerSSq));

  private FlipState flipState = FlipState.In;
  private double flipZeroRot = 0.0;
  private boolean interlockEnabled = false;
  private boolean flipManualOverride = false;
  private final Mechanism2d mech = new Mechanism2d(2.0, 1.0);
  private final MechanismLigament2d arm = mech.getRoot("Base", 0.5, 0.5)
      .append(new MechanismLigament2d("Arm", 0.4, 0, 6, new Color8Bit(Color.kYellow)));

  public IntakeSubsystem() {
    var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration()
        .withSlot0(new com.ctre.phoenix6.configs.Slot0Configs()
            .withKP(Constants.IntakeConstants.flipkP)
            .withKI(Constants.IntakeConstants.flipkI)
            .withKD(Constants.IntakeConstants.flipkD));
    flipMotor.getConfigurator().apply(cfg);
    flipMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    flipZeroRot = flipMotor.getPosition().getValueAsDouble();
    flipPid.reset(0.0);
    flipPid.setGoal(Constants.IntakeConstants.flipInPositionRot);
    SmartDashboard.putData("IntakeViz", mech);
  }

  public void setFlip(FlipState s) {
    if (interlockEnabled && s == FlipState.Out) {
      flipState = FlipState.In;
      flipPid.setGoal(Constants.IntakeConstants.flipInPositionRot);
      return;
    }
    flipState = s;
    double pos = s == FlipState.In ? Constants.IntakeConstants.flipInPositionRot
        : Constants.IntakeConstants.flipOutPositionRot;
    flipPid.setGoal(pos);
  }

  public void toggleFlip() {
    setFlip(flipState == FlipState.In ? FlipState.Out : FlipState.In);
  }

  public void runRollerForward() {
    rollerMotor.setControl(rollerCtrl.withOutput(Constants.IntakeConstants.rollerVoltage));
  }

  public void runRollerReverse() {
    rollerMotor.setControl(rollerCtrl.withOutput(-Constants.IntakeConstants.rollerVoltage));
  }

  public void stopRoller() {
    rollerMotor.setControl(rollerCtrl.withOutput(0));
  }

  private double flipTargetRot() {
    return flipState == FlipState.In ? Constants.IntakeConstants.flipInPositionRot
        : Constants.IntakeConstants.flipOutPositionRot;
  }

  private boolean flipAtTarget() {
    double pos = flipMotor.getPosition().getValueAsDouble() - flipZeroRot;
    return Math.abs(pos - flipTargetRot()) <= Constants.IntakeConstants.flipToleranceRot;
  }

  @Override
  public void periodic() {
    double currentRot = flipMotor.getPosition().getValueAsDouble() - flipZeroRot;
    if (!flipManualOverride) {
      double desiredVel = flipPid.getSetpoint().velocity;
      double volts = flipPid.calculate(currentRot) + Constants.IntakeConstants.flipkV * desiredVel;
      if (volts > 12.0)
        volts = 12.0;
      if (volts < -12.0)
        volts = -12.0;
      flipMotor.setControl(flipVoltageCtrl.withOutput(volts));
    }
    SmartDashboard.putNumber("Intake/FlipRot", currentRot);
    SmartDashboard.putNumber("Intake/FlipGoalRot", flipTargetRot());
    SmartDashboard.putBoolean("Intake/FlipAtTarget", flipAtTarget());
    if (flipManualOverride) {
      SmartDashboard.putNumber("Intake/CapturedFlipOutRot", currentRot);
    }
    double angleDeg = Math.toDegrees(currentRot / Constants.IntakeConstants.flipOutPositionRot * Math.PI / 2.0);
    arm.setAngle(angleDeg);
  }

  public void setInterlockEnabled(boolean enabled) {
    this.interlockEnabled = enabled;
  }

  private final SysIdRoutine sysIdFlip = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (volts) -> flipMotor.setControl(new VoltageOut(volts)),
          (log) -> {
            log.motor("intakeFlip")
                .voltage(Units.Volts.of(flipMotor.getMotorVoltage().getValueAsDouble()))
                .angularPosition(Units.Rotations.of(flipMotor.getPosition().getValueAsDouble()))
                .angularVelocity(Units.RotationsPerSecond.of(flipMotor.getVelocity().getValueAsDouble()));
          },
          this));

  public edu.wpi.first.wpilibj2.command.Command sysIdQuasistaticForward() {
    return sysIdFlip.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdQuasistaticReverse() {
    return sysIdFlip.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdDynamicForward() {
    return sysIdFlip.dynamic(SysIdRoutine.Direction.kForward);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdDynamicReverse() {
    return sysIdFlip.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command intakeCollect() {
    return runOnce(() -> setFlip(FlipState.Out))
        .andThen(Commands.waitUntil(this::flipAtTarget))
        .andThen(run(this::runRollerForward))
        .finallyDo(this::stopRoller);
  }

  public Command stowIntake() {
    return runOnce(() -> setFlip(FlipState.In)).finallyDo(this::stopRoller);
  }

  public Command runRollerForwardCommand() {
    return run(this::runRollerForward).finallyDo(this::stopRoller);
  }

  public Command runRollerReverseCommand() {
    return run(this::runRollerReverse).finallyDo(this::stopRoller);
  }

  public void setFlipManualVoltage(double volts) {
    flipManualOverride = true;
    flipMotor.setControl(flipVoltageCtrl.withOutput(volts));
  }

  public void stopFlipManual() {
    flipMotor.setControl(flipVoltageCtrl.withOutput(0));
    flipManualOverride = false;
  }

  public Command flipManualForwardCommand(double volts) {
    return run(() -> setFlipManualVoltage(Math.max(0.0, Math.min(12.0, volts)))).finallyDo(this::stopFlipManual);
  }

  public Command flipManualReverseCommand(double volts) {
    return run(() -> setFlipManualVoltage(-Math.max(0.0, Math.min(12.0, volts)))).finallyDo(this::stopFlipManual);
  }

  public Command captureFlipOutRotCommand() {
    return runOnce(() -> SmartDashboard.putNumber("Intake/CapturedFlipOutRot",
        flipMotor.getPosition().getValueAsDouble() - flipZeroRot));
  }
}
