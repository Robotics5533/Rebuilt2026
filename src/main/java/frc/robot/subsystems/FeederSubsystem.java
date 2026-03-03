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
public class FeederSubsystem extends SubsystemBase {


    private final TalonFX leftFeeder = new TalonFX(Constants.ShooterConstants.leftFeederId);
    private final TalonFX rightFeeder = new TalonFX(Constants.ShooterConstants.rightFeederId);


    private final VoltageOut voltageCtrl = new VoltageOut(0);

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
        leftFeeder.setControl(voltageCtrl.withOutput(volts));
    }

    /**
     * Sets the voltage output for the right feeder motor.
     * Note: The right feeder motor typically spins in the opposite direction, hence the negative voltage.
     * @param volts The voltage to apply to the right feeder motor.
     */
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

    /**
     * Stops both feeder motors.
     */
    public void stopFeeders() {
        stopLeftFeeder();
        stopRightFeeder();
    }

    /**
     * Returns a command that runs the left feeder at a predefined voltage.
     * @return A command to run the left feeder.
     */
    public Command runLeftFeederCommand() {
        return run(() -> setLeftFeederVoltage(Constants.ShooterConstants.feederVoltage));
    }

    /**
     * Returns a command that runs the right feeder at a predefined voltage.
     * @return A command to run the right feeder.
     */
    public Command runRightFeederCommand() {
        return run(() -> setRightFeederVoltage(Constants.ShooterConstants.feederVoltage));
    }

    /**
     * Returns a command that runs both feeders at a predefined voltage.
     * @return A command to run both feeders.
     */
    public Command runBothFeedersCommand() {
        return run(() -> {
            setLeftFeederVoltage(Constants.ShooterConstants.feederVoltage);
            setRightFeederVoltage(Constants.ShooterConstants.feederVoltage);
        });
    }
     public Command InvertrunBothFeedersCommand() {
        return run(() -> {
            setLeftFeederVoltage(-Constants.ShooterConstants.feederVoltage);
            setRightFeederVoltage(-Constants.ShooterConstants.feederVoltage);
        });
    }

    /**
     * Returns a command that stops the left feeder.
     * @return A command to stop the left feeder.
     */
    public Command stopLeftFeederCommand() {
        return runOnce(this::stopLeftFeeder);
    }

    /**
     * Returns a command that stops the right feeder.
     * @return A command to stop the right feeder.
     */
    public Command stopRightFeederCommand() {
        return runOnce(this::stopRightFeeder);
    }

    /**
     * Returns a command that stops both feeders.
     * @return A command to stop both feeders.
     */
    public Command stopFeedersCommand() {
        return runOnce(this::stopFeeders);
    }

    /**
     * Called periodically by the scheduler.
     * Updates SmartDashboard with current feeder motor voltages.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/LeftVoltage", leftFeeder.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/RightVoltage", rightFeeder.getMotorVoltage().getValueAsDouble());
    }
}
