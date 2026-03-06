package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX flipMotor =
      new TalonFX(Constants.IntakeConstants.intakeFlipMotorId);

  private final TalonFX rollerMotor =
      new TalonFX(Constants.IntakeConstants.intakeRollerMotorId);
  private final CANcoder flipEncoder =
      new CANcoder(Constants.IntakeConstants.intakeFlipEncoderId);

  private final VoltageOut flipVoltageCtrl = new VoltageOut(0);
  private final VoltageOut rollerCtrl = new VoltageOut(0);

  private double manualVoltage = 0.0;

  private final PositionVoltage positionOut = new PositionVoltage(0);

  public IntakeSubsystem() {
    var flipCfg = new TalonFXConfiguration();

    flipCfg.CurrentLimits.SupplyCurrentLimit =
        Constants.IntakeConstants.flipCurrentLimit;
    flipCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    flipCfg.CurrentLimits.StatorCurrentLimit =
        Constants.IntakeConstants.flipCurrentLimit;
    flipCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    flipMotor.getConfigurator().apply(flipCfg);
    flipMotor.setNeutralMode(NeutralModeValue.Coast);

    var rollerCfg = new TalonFXConfiguration();
    rollerCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCfg.CurrentLimits.StatorCurrentLimit = 30.0;
    rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerCfg);
    rollerMotor.setNeutralMode(NeutralModeValue.Coast);

    flipCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flipCfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Use cosine gravity compensation
    flipCfg.Slot0.kG = Constants.IntakeConstants.canflipkG; // Gravity gain
    flipCfg.Slot0.kS = Constants.IntakeConstants.canflipkS; // Static gain
    flipCfg.Slot0.kP = Constants.IntakeConstants.canflipkP; // Proportional gain
    flipCfg.Slot0.kD = Constants.IntakeConstants.canflipkD; // Derivative gain
    flipCfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // Configure the leader motor to use the CANcoder for position feedback
    flipCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    flipCfg.Feedback.FeedbackRemoteSensorID = flipEncoder.getDeviceID();
    // Try to apply config multiple time. Break after successfully applying
    for (int i = 0; i < 2; ++i) {
      var status = flipMotor.getConfigurator().apply(flipCfg);
      if (status.isOK()) break;
    }

  }

  @Override
  public void periodic() {
    flipMotor.setControl(flipVoltageCtrl.withOutput(manualVoltage));

    SmartDashboard.putNumber("Intake/FlipDeg",
        Units.rotationsToDegrees(
            flipMotor.getPosition().getValueAsDouble()));

    SmartDashboard.putNumber("Intake/FlipAppliedVolts",
        manualVoltage);
  }

  /* ---------------- Flip Voltage Control ---------------- */

  public void setFlipVoltage(double volts) {
    manualVoltage = Math.max(-12.0, Math.min(12.0, volts));
  }
  
  public void setPosition(double position) {
    // Apply the position output to the flip motor
    flipMotor.setControl(positionOut.withPosition(position));
  }

   public Command outpos() {
    // Command to run the arm to vertical position and stop it afterward
    return runOnce(() -> setPosition(Constants.IntakeConstants.flipOutPositionDeg / 360.0));
  }

  public Command inpos() {
    // Command to run the arm to horizontal position and stop it afterward
    return runOnce(() -> setPosition(Constants.IntakeConstants.flipInPositionDeg / 360.0));
  }


  public void stopFlip() {
    manualVoltage = 0.0;
  }

  public Command flipManualForwardCommand(double volts) {
    return run(() -> setFlipVoltage(Math.abs(volts)))
        .finallyDo(this::stopFlip);
  }

  public Command flipManualReverseCommand(double volts) {
    return run(() -> setFlipVoltage(-Math.abs(volts)))
        .finallyDo(this::stopFlip);
  }

  /* ---------------- Roller Control ---------------- */

  public void runRollerForward() {
    rollerMotor.setControl(
        rollerCtrl.withOutput(Constants.IntakeConstants.rollerVoltage));
  }

  public void runRollerReverse() {
    rollerMotor.setControl(
        rollerCtrl.withOutput(-Constants.IntakeConstants.rollerVoltage));
  }

  public void stopRoller() {
    rollerMotor.setControl(rollerCtrl.withOutput(0));
  }

  public Command runRollerForwardCommand() {
    return run(this::runRollerForward)
        .finallyDo(this::stopRoller);
  }

  public Command runRollerReverseCommand() {
    return run(this::runRollerReverse)
        .finallyDo(this::stopRoller);
  }
}
