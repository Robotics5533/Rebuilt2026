



package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeFlipConstants;

public class IntakeFlip extends SubsystemBase {

  private final TalonFX flipMotor = new TalonFX(IntakeFlipConstants.flipMotorId);
  private final CANcoder flipEncoder = new CANcoder(IntakeFlipConstants.flipEncoderId);

  private final PositionVoltage positionOut = new PositionVoltage(0);
  private final VoltageOut voltageOut = new VoltageOut(0);

  private double goalPositionRotations = 0.0;


  /** Creates a new IntakeFlip. */
  public IntakeFlip() {
    configureHardware();

    var absPosSignal = flipEncoder.getAbsolutePosition();
    absPosSignal.waitForUpdate(0.1); 

    if (absPosSignal.getStatus().isOK()) {
      double currentRotations = absPosSignal.getValueAsDouble();
      double outRotations = Units.degreesToRotations(IntakeFlipConstants.flipOutPositionDeg);
      double inRotations = Units.degreesToRotations(IntakeFlipConstants.flipInPositionDeg);

      if (Math.abs(currentRotations - outRotations) < Math.abs(currentRotations - inRotations)) {
        goalPositionRotations = outRotations;
      } else {
        goalPositionRotations = inRotations;
      }
    } else {
      goalPositionRotations = Units.degreesToRotations(IntakeFlipConstants.flipOutPositionDeg);
    }

    
    stop();
  }

  private void configureHardware() {
    TalonFXConfiguration flipConfig = new TalonFXConfiguration();
    flipConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    flipConfig.Slot0.kP = IntakeFlipConstants.flipkP;
    flipConfig.Slot0.kI = IntakeFlipConstants.flipkI;
    flipConfig.Slot0.kD = IntakeFlipConstants.flipkD;
    flipConfig.Slot0.kV = IntakeFlipConstants.flipkV;
    flipConfig.Slot0.kA = IntakeFlipConstants.flipkA;
    flipConfig.Slot0.kG = IntakeFlipConstants.flipkG;
    flipConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    flipConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    flipConfig.Feedback.FeedbackRemoteSensorID = flipEncoder.getDeviceID();
    flipConfig.Feedback.SensorToMechanismRatio = IntakeFlipConstants.flipGearRatio;
    flipConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(IntakeFlipConstants.softLimitForwardDeg);
    flipConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    flipConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(IntakeFlipConstants.softLimitReverseDeg);
    flipConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    var status = flipMotor.getConfigurator().apply(flipConfig);
    if (!status.isOK()) {
      System.out.println("Failed to configure IntakeFlip motor");
    }

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    var encoderStatus = flipEncoder.getConfigurator().apply(cancoderConfig);
    if (!encoderStatus.isOK()) {
      System.out.println("Failed to configure IntakeFlip encoder");
    }
  }

  /**
   * Sets the position for the intake flip.
   *
   * @param position The position to set in rotations.
   */
  public void setPosition(double position) {
    this.goalPositionRotations = position;
    flipMotor.setControl(positionOut.withPosition(position));
  }

  public Command setPositionCommand(double position) {
    return runOnce(() -> setPosition(position));
  }

  public Command inPosition() {
    return setPositionCommand(Units.degreesToRotations(IntakeFlipConstants.flipInPositionDeg));
  }

  public Command outPosition() {
    return setPositionCommand(Units.degreesToRotations(IntakeFlipConstants.flipOutPositionDeg));
  }

  public void stop() {
    flipMotor.stopMotor();
  }

  public boolean isAtInPosition() {
    return Math.abs(flipMotor.getPosition().getValueAsDouble() - Units.degreesToRotations(IntakeFlipConstants.flipInPositionDeg)) < Units.degreesToRotations(IntakeFlipConstants.flipToleranceDeg);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Override
  public void periodic() {
    double currentPosition = flipMotor.getPosition().getValueAsDouble();
    double error = goalPositionRotations - currentPosition;

    SmartDashboard.putNumber("IntakeFlip/PositionDeg", Units.rotationsToDegrees(currentPosition));
    SmartDashboard.putNumber("IntakeFlip/GoalPositionDeg", Units.rotationsToDegrees(goalPositionRotations));
    SmartDashboard.putNumber("IntakeFlip/ErrorDeg", Units.rotationsToDegrees(error));
    SmartDashboard.putNumber("IntakeFlip/AppliedVoltage", flipMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("IntakeFlip/CurrentAmps", flipMotor.getSupplyCurrent().getValueAsDouble());
  }

  /**
   * Manually runs the intake flip mechanism at a given voltage.
   *
   * @param voltage The voltage to apply to the motor.
   */
  public void runManual(double voltage) {
    flipMotor.setControl(voltageOut.withOutput(voltage));
  }

  /**
   * Stops the intake flip mechanism when in manual mode.
   */
  public void stopManual() {
    flipMotor.stopMotor();
  }
}
