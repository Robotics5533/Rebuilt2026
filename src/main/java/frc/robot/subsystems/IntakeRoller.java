



package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;

public class IntakeRoller extends SubsystemBase {
  private final TalonFX rollerMotor = new TalonFX(IntakeRollerConstants.rollerMotorId);
  private final VoltageOut rollerCtrl = new VoltageOut(0);

  /** Creates a new IntakeRoller. */
  public IntakeRoller() {
    configureHardware();
  }

  private void configureHardware() {
    TalonFXConfiguration rollerCfg = new TalonFXConfiguration();
    rollerCfg.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.rollerCurrentLimit;
    rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCfg.CurrentLimits.StatorCurrentLimit = IntakeRollerConstants.rollerCurrentLimit;
    rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerCfg);
    rollerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void runRollerForward() {
    rollerMotor.setControl(rollerCtrl.withOutput(IntakeRollerConstants.rollerVoltage));
  }

  public void runRollerReverse() {
    rollerMotor.setControl(rollerCtrl.withOutput(-IntakeRollerConstants.rollerVoltage));
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeRoller/AppliedVoltage", rollerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("IntakeRoller/CurrentAmps", rollerMotor.getSupplyCurrent().getValueAsDouble());
  }
}
