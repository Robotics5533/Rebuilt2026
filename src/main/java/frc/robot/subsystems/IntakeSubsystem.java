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

  private final TalonFX flipMotor = new TalonFX(Constants.IntakeConstants.intakeFlipMotorId);

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
  private Double capturedManualPosition = null;

  public IntakeSubsystem() {
    var cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.flipGearRatio;

    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.flipCurrentLimit;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.flipCurrentLimit;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
        .degreesToRotations(Constants.IntakeConstants.softLimitForwardDeg);
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
        .degreesToRotations(Constants.IntakeConstants.softLimitReverseDeg);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    flipMotor.getConfigurator().apply(cfg);
    flipMotor.setNeutralMode(NeutralModeValue.Brake);
    double inPosRot = Units.degreesToRotations(Constants.IntakeConstants.flipInPositionDeg);
    flipMotor.setPosition(inPosRot);
    flipController.reset(inPosRot);

    flipController.setTolerance(Units.degreesToRotations(Constants.IntakeConstants.flipToleranceDeg));
  }

  public void setFlip(FlipState s) {
    if (interlockEnabled && s == FlipState.Out) {
      flipState = FlipState.In;
      return;
    }
    flipState = s;
    capturedManualPosition = null; // Clear manual hold, go back to preset position
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
    return flipMotor.getPosition().getValueAsDouble();
  }

  public double getFlipPositionDeg() {
    return Units.rotationsToDegrees(getFlipPositionRot());
  }

  @Override
  public void periodic() {
    double currentRot = getFlipPositionRot();
    double targetRot = (capturedManualPosition != null) 
        ? capturedManualPosition 
        : Units.degreesToRotations(flipState.angleDeg);

    double pidOutput = flipController.calculate(currentRot, targetRot);

    double gravityAssist = Constants.IntakeConstants.flipGravityAssistVoltage;

    double totalVoltage = pidOutput + gravityAssist;

    flipMotor.setControl(flipVoltageCtrl.withOutput(totalVoltage));

    SmartDashboard.putNumber("Intake/FlipDeg", Units.rotationsToDegrees(currentRot));
    SmartDashboard.putNumber("Intake/FlipGoalDeg", capturedManualPosition != null ? Units.rotationsToDegrees(capturedManualPosition) : flipState.angleDeg);
    SmartDashboard.putNumber("Intake/FlipAppliedVolts", totalVoltage);
    SmartDashboard.putBoolean("Intake/FlipAtTarget", flipAtTarget());
  }

  public void setInterlockEnabled(boolean enabled) {
    this.interlockEnabled = enabled;
  }
  

  
  public void setFlipManualVoltage(double volts) {
    flipMotor.setControl(flipVoltageCtrl.withOutput(volts));
  }

  public void stopFlipManual() {
    // Capture current position so it holds here instead of going back to flipState
    double currentRot = getFlipPositionRot();
    capturedManualPosition = currentRot;
    flipController.reset(currentRot);
    flipController.setGoal(new TrapezoidProfile.State(currentRot, 0));
  }

  public Command flipManualForwardCommand(double volts) {
    return run(() -> setFlipManualVoltage(Math.max(0.0, Math.min(Constants.IntakeConstants.manualFlipVoltage, volts)))).finallyDo(this::stopFlipManual);
  }

  public Command flipManualReverseCommand(double volts) {
    return run(() -> setFlipManualVoltage(-Math.max(0.0, Math.min(Constants.IntakeConstants.manualFlipVoltage, volts)))).finallyDo(this::stopFlipManual);
  }

  public Command captureFlipOutRotCommand() {
    return runOnce(() -> SmartDashboard.putNumber("Intake/CapturedFlipOutDeg", getFlipPositionDeg()));
  }
}