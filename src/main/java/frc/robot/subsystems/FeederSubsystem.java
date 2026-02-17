package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private final TalonFX leftFeeder = new TalonFX(Constants.ShooterConstants.leftFeederId);
    private final TalonFX rightFeeder = new TalonFX(Constants.ShooterConstants.rightFeederId);

    private final VoltageOut voltageCtrl = new VoltageOut(0);

    public FeederSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.feederCurrentLimit;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.feederCurrentLimit;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        leftFeeder.getConfigurator().apply(cfg);
        rightFeeder.getConfigurator().apply(cfg);

        leftFeeder.setNeutralMode(NeutralModeValue.Coast);
        rightFeeder.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setLeftFeederVoltage(double volts) {
        leftFeeder.setControl(voltageCtrl.withOutput(volts));
    }

    public void setRightFeederVoltage(double volts) {
        rightFeeder.setControl(voltageCtrl.withOutput(-volts));
    }

    public void stopLeftFeeder() {
        setLeftFeederVoltage(0);
    }

    public void stopRightFeeder() {
        setRightFeederVoltage(0);
    }

    public void stopFeeders() {
        stopLeftFeeder();
        stopRightFeeder();
    }

    public Command runLeftFeederCommand() {
        return run(() -> setLeftFeederVoltage(Constants.ShooterConstants.feederVoltage));
    }

    public Command runRightFeederCommand() {
        return run(() -> setRightFeederVoltage(Constants.ShooterConstants.feederVoltage));
    }

    public Command runBothFeedersCommand() {
        return run(() -> {
            setLeftFeederVoltage(Constants.ShooterConstants.feederVoltage);
            setRightFeederVoltage(Constants.ShooterConstants.feederVoltage);
        });
    }

    public Command stopLeftFeederCommand() {
        return runOnce(this::stopLeftFeeder);
    }

    public Command stopRightFeederCommand() {
        return runOnce(this::stopRightFeeder);
    }

    public Command stopFeedersCommand() {
        return runOnce(this::stopFeeders);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/LeftVoltage", leftFeeder.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/RightVoltage", rightFeeder.getMotorVoltage().getValueAsDouble());
    }
}
