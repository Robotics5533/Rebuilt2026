package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Represents the robot's feeder mechanism, responsible for moving game pieces
 * towards the shooter. This version uses velocity (RPS) control for more precise feeding.
 */
public class FeederRPSSubsystem extends SubsystemBase implements IFeederSubsystem {

    private final TalonFX leftFeeder = new TalonFX(Constants.ShooterConstants.leftFeederId);
    private final TalonFX rightFeeder = new TalonFX(Constants.ShooterConstants.rightFeederId);

    private final VelocityVoltage velocityCtrl = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageCtrl = new VoltageOut(0);

    private double targetRPS = 0.0;

    /**
     * Constructs a new FeederRPSSubsystem.
     * Configures the TalonFX motor controllers for velocity control.
     */
    public FeederRPSSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.feederCurrentLimit;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.feederCurrentLimit;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.FeederConstants.FEEDER_RPS_KP;
        slot0Configs.kI = Constants.FeederConstants.FEEDER_RPS_KI;
        slot0Configs.kD = Constants.FeederConstants.FEEDER_RPS_KD;
        slot0Configs.kS = Constants.FeederConstants.FEEDER_RPS_KS;
        slot0Configs.kV = Constants.FeederConstants.FEEDER_RPS_KV;
        slot0Configs.kA = Constants.FeederConstants.FEEDER_RPS_KA;

        leftFeeder.getConfigurator().apply(cfg);
        rightFeeder.getConfigurator().apply(cfg);
        leftFeeder.getConfigurator().apply(slot0Configs);
        rightFeeder.getConfigurator().apply(slot0Configs);
    }

    /**
     * Sets the target RPS for the feeder motors.
     * @param rps The desired rotations per second.
     */
    public void setTargetRPS(double rps) {
        this.targetRPS = rps;
        double targetRotationsPer100ms = rps / 10.0; 
        leftFeeder.setControl(velocityCtrl.withVelocity(targetRotationsPer100ms));
        rightFeeder.setControl(velocityCtrl.withVelocity(-targetRotationsPer100ms));
    }

    /**
     * Gets the current RPS of the left feeder motor.
     * @return The current rotations per second.
     */
    public double getLeftFeederRPS() {
        return leftFeeder.getVelocity().getValueAsDouble() * 10.0;
    }

    /**
     * Gets the current RPS of the right feeder motor.
     * @return The current rotations per second.
     */
    public double getRightFeederRPS() {
        // Return as positive RPS for consistency, even if motor is spinning negatively
        return Math.abs(rightFeeder.getVelocity().getValueAsDouble() * 10.0);
    }

    /**
     * Checks if both feeder motors are at the target RPS within a tolerance.
     * @return True if both feeders are at target speed, false otherwise.
     */
    public boolean areFeedersAtSpeed() {
        return Math.abs(getLeftFeederRPS() - targetRPS) < Constants.FeederConstants.FEEDER_RPS_TOLERANCE &&
               Math.abs(getRightFeederRPS() - targetRPS) < Constants.FeederConstants.FEEDER_RPS_TOLERANCE;
    }

    @Override
    public Command runFeedersCommand() {
        return runOnce(() -> setTargetRPS(Constants.FeederConstants.FEEDER_RPS_SETPOINT));
    }

    @Override
    public double getLeftFeederVoltage() {
        return 0.0;
    }

    @Override
    public double getRightFeederVoltage() {
        return 0.0;
    }

    @Override
    public double getTargetVoltage() {
        return 0.0;
    }

    @Override
    public double getTargetRPS() {
        return targetRPS;
    }


    /**
     * Stops both feeder motors.
     */
    public void stopFeeders() {
        leftFeeder.setControl(voltageCtrl.withOutput(0));
        rightFeeder.setControl(voltageCtrl.withOutput(0));
        targetRPS = 0.0;
    }

    /**
     * Returns a command that sets the feeder to a specific RPS.
     * @param rps The target RPS.
     * @return A command to set the feeder RPS.
     */
    public Command setFeederRPSCommand(double rps) {
        return runOnce(() -> setTargetRPS(rps));
    }

    /**
     * Returns a command that stops the feeders.
     * @return A command to stop the feeders.
     */
    public Command stopFeedersCommand() {
        return runOnce(this::stopFeeders);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FeederRPS/TargetRPS", targetRPS);
        SmartDashboard.putNumber("FeederRPS/LeftRPS", getLeftFeederRPS());
        SmartDashboard.putNumber("FeederRPS/RightRPS", getRightFeederRPS());
        SmartDashboard.putBoolean("FeederRPS/AtTargetSpeed", areFeedersAtSpeed());
    }
}
