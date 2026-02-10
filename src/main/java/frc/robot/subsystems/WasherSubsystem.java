package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WasherSubsystem extends SubsystemBase {
    private final TalonFX leftWasher = new TalonFX(Constants.ShooterConstants.leftWasherMotorId);
    private final TalonFX rightWasher = new TalonFX(Constants.ShooterConstants.rightWasherMotorId);

    private final VoltageOut voltageCtrl = new VoltageOut(0);

    public WasherSubsystem() {
        leftWasher.setNeutralMode(NeutralModeValue.Brake);
        rightWasher.setNeutralMode(NeutralModeValue.Brake);
    }

    public void runWasher(double volts) {
        leftWasher.setControl(voltageCtrl.withOutput(volts));
        rightWasher.setControl(voltageCtrl.withOutput(-volts));
    }

    public void stopWasher() {
        leftWasher.setControl(voltageCtrl.withOutput(0));
        rightWasher.setControl(voltageCtrl.withOutput(0));
    }

    public Command run(double volts) {
        return run(() -> runWasher(volts)).finallyDo(this::stopWasher);
    }
}
