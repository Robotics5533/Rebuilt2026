package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;

@Logged
public class RunRollers extends SubsystemBase {

  private final TalonFX rollerMotor =
      new TalonFX(Constants.IntakeConstants.intakeRollerMotorId);


  private final VoltageOut rollerCtrl = new VoltageOut(0);


  public RunRollers() {
    var rollerCfg = new TalonFXConfiguration();
    rollerCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCfg.CurrentLimits.StatorCurrentLimit = 30.0;
    rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerCfg);
    rollerMotor.setNeutralMode(NeutralModeValue.Coast);
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