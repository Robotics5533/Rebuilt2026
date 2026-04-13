package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class IntakeSubsystem extends SubsystemBase {
  public enum FlipState {
    In(Constants.IntakeConstants.flipInPositionDeg),
    Out(Constants.IntakeConstants.flipOutPositionDeg),
    Erect(Constants.IntakeConstants.flipErectPositionDeg);

    public final double angleDeg;

    FlipState(double angleDeg) {
      this.angleDeg = angleDeg;
    }

    public Angle angle() {
      return Degrees.of(angleDeg);
    }
  }

  // Motors - right runs clockwise, left runs counter-clockwise
  private final TalonFX RightflipMotor = new TalonFX(Constants.IntakeConstants.intakeRightFlipMotorId);
  private final TalonFX LeftflipMotor = new TalonFX(Constants.IntakeConstants.intakeLeftFlipMotorId);

  private final VoltageOut flipVoltageCtrl = new VoltageOut(0);

  // PID controller for position tracking
  private final ProfiledPIDController flipController = new ProfiledPIDController(
      Constants.IntakeConstants.flipkP,
      Constants.IntakeConstants.flipkI,
      Constants.IntakeConstants.flipkD,
      new TrapezoidProfile.Constraints(
          Constants.IntakeConstants.flipMaxVelocityRotPerS,
          Constants.IntakeConstants.flipMaxAccelRotPerSSq));

  private FlipState flipState = FlipState.In;
  private boolean interlockEnabled = false;
  private Double capturedManualPosition = null;

  public IntakeSubsystem() {
    // Configure base settings for both motors
    var baseCfg = new TalonFXConfiguration();
    baseCfg.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.flipGearRatio;
    baseCfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.flipCurrentLimit;
    baseCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    baseCfg.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.flipCurrentLimit;
    baseCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    baseCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
        .degreesToRotations(Constants.IntakeConstants.softLimitForwardDeg);
    baseCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    baseCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
        .degreesToRotations(Constants.IntakeConstants.softLimitReverseDeg);
    baseCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // Configure right motor (clockwise positive)
    RightflipMotor.getConfigurator().apply(baseCfg);
    RightflipMotor.setNeutralMode(NeutralModeValue.Brake);
    // Reset position to 0 first to ensure clean startup
    RightflipMotor.setPosition(0);

    // Configure left motor (counter-clockwise positive)
    LeftflipMotor.getConfigurator().apply(baseCfg);
    LeftflipMotor.setNeutralMode(NeutralModeValue.Brake);
    // Reset position to 0 first to ensure clean startup
    LeftflipMotor.setPosition(0);

    // Initialize position to "In" for both motors
    double inPosRot = Units.degreesToRotations(Constants.IntakeConstants.flipInPositionDeg);
    RightflipMotor.setPosition(inPosRot);
    LeftflipMotor.setPosition(inPosRot);

    // Initialize PID controller
    flipController.reset(inPosRot);
    flipController.setTolerance(Units.degreesToRotations(Constants.IntakeConstants.flipToleranceDeg));
  }

  public void setFlip(FlipState s) {
    // Prevent flipping out if safety interlock is enabled
    if (interlockEnabled && s == FlipState.Out) {
      flipState = FlipState.In;
      return;
    }

    flipState = s;
    capturedManualPosition = null;
  }

  public void toggleFlip() {
    setFlip(flipState == FlipState.In ? FlipState.Out : FlipState.In);
  }

  public void erect() {
    setFlip(FlipState.Erect);
  }

  public void outFlip() {
    setFlip(FlipState.Out);
  }

  private boolean flipAtTarget() {
    double currentRot = getFlipPositionRot();
    double targetRot = Units.degreesToRotations(flipState.angleDeg);
    return Math.abs(currentRot - targetRot) <= Units.degreesToRotations(Constants.IntakeConstants.flipToleranceDeg);
  }

  private double getFlipPositionRot() {
    // Use average of both motors for position feedback
    double rightPos = RightflipMotor.getPosition().getValueAsDouble();
    double leftPos = LeftflipMotor.getPosition().getValueAsDouble();
    return (rightPos + leftPos) / 2.0;
  }

  public double getFlipPositionDeg() {
    return Units.rotationsToDegrees(getFlipPositionRot());
  }

  @Override
  public void periodic() {
    // Get current position in rotations
    double currentRot = getFlipPositionRot();

    // Use manual position if set, otherwise use target flip state position
    double targetRot = (capturedManualPosition != null)
        ? capturedManualPosition
        : Units.degreesToRotations(flipState.angleDeg);

    // Calculate PID output for smooth motion
    double pidOutput = flipController.calculate(currentRot, targetRot);

    // Add gravity assist to help counteract weight
    double gravityAssist = Constants.IntakeConstants.flipGravityAssistVoltage;

    // Combine PID output with gravity assist
    double totalVoltage = pidOutput + gravityAssist;

    // Apply voltage to both motors
    // Right motor: positive voltage = clockwise
    // Left motor: negative voltage = counter-clockwise (they face opposite directions)
    RightflipMotor.setControl(flipVoltageCtrl.withOutput(totalVoltage));
    LeftflipMotor.setControl(flipVoltageCtrl.withOutput(-totalVoltage));

    // Update SmartDashboard for debugging and monitoring
    SmartDashboard.putNumber("Intake/FlipDeg", Units.rotationsToDegrees(currentRot));
    SmartDashboard.putNumber("Intake/FlipGoalDeg",
        capturedManualPosition != null ? Units.rotationsToDegrees(capturedManualPosition) : flipState.angleDeg);
    SmartDashboard.putNumber("Intake/FlipAppliedVolts", totalVoltage);
    SmartDashboard.putBoolean("Intake/FlipAtTarget", flipAtTarget());
  }

  public void setInterlockEnabled(boolean enabled) {
    this.interlockEnabled = enabled;
  }

  public void setFlipManualVoltage(double volts) {
    // Right motor: positive voltage = clockwise
    // Left motor: negative voltage = counter-clockwise (opposite direction)
    RightflipMotor.setControl(flipVoltageCtrl.withOutput(volts));
    LeftflipMotor.setControl(flipVoltageCtrl.withOutput(-volts));
  }

  public void stopFlipManual() {
    // Capture current position so it holds here instead of going back to flipState
    double currentRot = getFlipPositionRot();
    capturedManualPosition = currentRot;
    flipController.reset(currentRot);
    flipController.setGoal(new TrapezoidProfile.State(currentRot, 0));
  }

  public Command flipManualForwardCommand(double volts) {
    return run(() -> setFlipManualVoltage(Math.max(0.0, Math.min(Constants.IntakeConstants.manualFlipVoltage, volts))))
        .finallyDo(this::stopFlipManual);
  }

  public Command flipManualReverseCommand(double volts) {
    return run(() -> setFlipManualVoltage(-Math.max(0.0, Math.min(Constants.IntakeConstants.manualFlipVoltage, volts))))
        .finallyDo(this::stopFlipManual);
  }

  public Command captureFlipOutRotCommand() {
    return runOnce(() -> SmartDashboard.putNumber("Intake/CapturedFlipOutDeg", getFlipPositionDeg()));
  }
}