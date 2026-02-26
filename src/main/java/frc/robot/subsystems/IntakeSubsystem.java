package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;

public class IntakeSubsystem extends SubsystemBase {
  public enum FlipState {
    In(Constants.IntakeConstants.flipInPositionDeg),
    Out(Constants.IntakeConstants.flipOutPositionDeg);

    public final double angleDeg;

    FlipState(double angleDeg) {
      this.angleDeg = angleDeg;
    }

    public Angle angle() {
      return Degrees.of(angleDeg);
    }
  }

  private final TalonFX flipMotor = new TalonFX(Constants.IntakeConstants.intakeFlipMotorId);
  private final TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.intakeRollerMotorId);
  private final CANcoder flipCANCoder = new CANcoder(Constants.IntakeConstants.intakeFlipEncoder);

  private final VoltageOut rollerCtrl = new VoltageOut(0);
  private final VoltageOut flipVoltageCtrl = new VoltageOut(0);

  private final ProfiledPIDController flipController = new ProfiledPIDController(
      Constants.IntakeConstants.flipkP,
      Constants.IntakeConstants.flipkI,
      Constants.IntakeConstants.flipkD,
      new TrapezoidProfile.Constraints(
          Constants.IntakeConstants.flipMaxVelocityRotPerS,
          Constants.IntakeConstants.flipMaxAccelRotPerSSq));

  private FlipState flipState = FlipState.In;
  private boolean interlockEnabled = false;
  private boolean flipManualOverride = false;

  public IntakeSubsystem() {
    var motorCfg = new TalonFXConfiguration();

    motorCfg.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.flipGearRatio;

    motorCfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.flipCurrentLimit;
    motorCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorCfg.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.flipCurrentLimit;
    motorCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    motorCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
        .degreesToRotations(Constants.IntakeConstants.softLimitForwardDeg);
    motorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
        .degreesToRotations(Constants.IntakeConstants.softLimitReverseDeg);
    motorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    flipMotor.getConfigurator().apply(motorCfg);
    flipMotor.setNeutralMode(NeutralModeValue.Coast);

    var canCoderCfg = new CANcoderConfiguration();
    canCoderCfg.MagnetSensor.MagnetOffset = Constants.IntakeConstants.flipCANCoderOffset;
    flipCANCoder.getConfigurator().apply(canCoderCfg);

    double initialCANCoderRotations = Units.degreesToRotations(flipCANCoder.getAbsolutePosition().getValueAsDouble());
    
    flipMotor.setPosition(initialCANCoderRotations);
    flipController.reset(initialCANCoderRotations);

    flipController.setTolerance(Units.degreesToRotations(Constants.IntakeConstants.flipToleranceDeg));

    var rollerCfg = new TalonFXConfiguration();
    rollerCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCfg.CurrentLimits.StatorCurrentLimit = 30.0;
    rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerCfg);
    rollerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setFlip(FlipState s) {
    if (interlockEnabled && s == FlipState.Out) {
      flipState = FlipState.In;
      return;
    }
    flipState = s;
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

  private boolean flipAtTarget() {
    double currentRot = getFlipPositionRot();
    double targetRot = Units.degreesToRotations(flipState.angleDeg);
    return Math.abs(currentRot - targetRot) <= Units.degreesToRotations(Constants.IntakeConstants.flipToleranceDeg);
  }

  private double getFlipPositionRot() {
    return flipMotor.getPosition().getValueAsDouble();
  }

  public double getFlipPositionDeg() {
    return Units.rotationsToDegrees(getFlipPositionRot());
  }

  @Override
  public void periodic() {
    double currentCANCoderDegrees = flipCANCoder.getAbsolutePosition().getValueAsDouble();
    double currentCANCoderRotations = Units.degreesToRotations(currentCANCoderDegrees);
    flipMotor.setPosition(currentCANCoderRotations);

    double currentRot = getFlipPositionRot();
    double targetRot = Units.degreesToRotations(flipState.angleDeg);

    double pidOutput = flipController.calculate(currentRot, targetRot);

    double totalVoltage = pidOutput;

    if (!flipManualOverride) {
      flipMotor.setControl(flipVoltageCtrl.withOutput(totalVoltage));
    }

    SmartDashboard.putNumber("Intake/FlipDeg", Units.rotationsToDegrees(currentRot));
    SmartDashboard.putNumber("Intake/FlipGoalDeg", flipState.angleDeg);
    SmartDashboard.putNumber("Intake/FlipAppliedVolts", totalVoltage);
    SmartDashboard.putBoolean("Intake/FlipAtTarget", flipAtTarget());
    SmartDashboard.putNumber("Intake/CANCoderDeg", currentCANCoderDegrees);
    SmartDashboard.putNumber("Intake/FlipVelocityRotPerSec", flipMotor.getVelocity().getValueAsDouble());

    if (flipManualOverride) {
      SmartDashboard.putNumber("Intake/CapturedFlipOutDeg", Units.rotationsToDegrees(currentRot));
      flipController.reset(currentRot);
    }
  }

  public void setInterlockEnabled(boolean enabled) {
    this.interlockEnabled = enabled;
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
    flipManualOverride = false;
    setFlip(flipState);
  }

  public Command flipManualForwardCommand(double volts) {
    return run(() -> setFlipManualVoltage(Math.max(0.0, Math.min(12.0, volts)))).finallyDo(this::stopFlipManual);
  }

  public Command flipManualReverseCommand(double volts) {
    return run(() -> setFlipManualVoltage(-Math.max(0.0, Math.min(12.0, volts)))).finallyDo(this::stopFlipManual);
  }

  public Command captureFlipOutRotCommand() {
    return runOnce(() -> SmartDashboard.putNumber("Intake/CapturedFlipOutDeg", getFlipPositionDeg()));
  }
}
