package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RunRollers extends SubsystemBase {

  private final TalonFX flipMotor =
      new TalonFX(Constants.IntakeConstants.intakeFlipMotorId);

  private final TalonFX rollerMotor =
      new TalonFX(Constants.IntakeConstants.intakeRollerMotorId);

  private final VoltageOut flipVoltageCtrl = new VoltageOut(0);
  private final VoltageOut rollerCtrl = new VoltageOut(0);

  private double manualVoltage = 0.0;

  public RunRollers() {
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
  }

  @Override
  public void periodic() {
    flipMotor.setControl(flipVoltageCtrl.withOutput(manualVoltage));

    SmartDashboard.putNumber("Intake/FlipDeg",
        Units.rotationsToDegrees(
            flipMotor.getPosition().getValueAsDouble()));
  }

  /* ---------------- Flip Voltage Control ---------------- */

  public void setFlipVoltage(double volts) {
    manualVoltage = Math.max(-12.0, Math.min(12.0, volts));
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