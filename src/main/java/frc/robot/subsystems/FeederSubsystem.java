package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Represents the robot's feeder mechanism, responsible for moving game pieces
 * towards the shooter. It controls two Falcon 500 (TalonFX) motors.
 */
public class FeederSubsystem extends SubsystemBase implements IFeederSubsystem {

    private final TalonFX leftFeeder = new TalonFX(Constants.ShooterConstants.leftFeederId);
    private final TalonFX rightFeeder = new TalonFX(Constants.ShooterConstants.rightFeederId);

    private final VoltageOut voltageCtrl = new VoltageOut(0);
    private double targetVoltage = 0.0;

    /**
     * Constructs a new FeederSubsystem.
     * Configures the TalonFX motor controllers with current limits and neutral mode.
     */
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


    /**
     * Sets the voltage output for the left feeder motor.
     * @param volts The voltage to apply to the left feeder motor.
     */
    public void setLeftFeederVoltage(double volts) {
        targetVoltage = volts;
        leftFeeder.setControl(voltageCtrl.withOutput(volts));
    }

    public void setRightFeederVoltage(double volts) {
        rightFeeder.setControl(voltageCtrl.withOutput(-volts));
    }

    /**
     * Stops the left feeder motor by setting its voltage to zero.
     */
    public void stopLeftFeeder() {
        setLeftFeederVoltage(0);
    }

    /**
     * Stops the right feeder motor by setting its voltage to zero.
     */
    public void stopRightFeeder() {
        setRightFeederVoltage(0);
    }

    @Override
    public void stopFeeders() {
        stopLeftFeeder();
        stopRightFeeder();
        targetVoltage = 0.0;
    }

    @Override
    public Command runFeedersCommand() {
        return run(() -> {
            setLeftFeederVoltage(Constants.ShooterConstants.feederVoltage);
            setRightFeederVoltage(Constants.ShooterConstants.feederVoltage);
        });
    }

    @Override
    public Command stopFeedersCommand() {
        return runOnce(this::stopFeeders);
    }

    @Override
    public boolean areFeedersAtSpeed() {
        return targetVoltage != 0.0;
    }

    @Override
    public double getLeftFeederVoltage() {
        return leftFeeder.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getRightFeederVoltage() {
        return rightFeeder.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getLeftFeederRPS() {
        return 0.0;
    }

    @Override
    public double getRightFeederRPS() {
        return 0.0;
    }

    @Override
    public double getTargetRPS() {
        return 0.0;
    }

    @Override
    public double getTargetVoltage() {
        return targetVoltage;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/LeftVoltage", leftFeeder.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/RightVoltage", rightFeeder.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/TargetVoltage", targetVoltage);
        SmartDashboard.putBoolean("Feeder/AreFeedersAtSpeed", areFeedersAtSpeed());
    }
}
