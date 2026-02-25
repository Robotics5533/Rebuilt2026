package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IFeederSubsystem extends Subsystem {

    /**
     * Returns a command that runs the feeders at their default or configured speed/voltage.
     * The specific value is determined by the implementation.
     * @return A command to run the feeders.
     */
    Command runFeedersCommand();

    /**
     * Returns a command that stops the feeders.
     * @return A command to stop the feeders.
     */
    Command stopFeedersCommand();

    /**
     * Stops the feeder motors directly (non-command).
     */
    void stopFeeders();

    /**
     * Checks if the feeder motors are at their target speed/setpoint.
     * @return True if the feeders are at the target, false otherwise.
     */
    boolean areFeedersAtSpeed();

    /**
     * Returns the current voltage being applied to the left feeder motor.
     * @return The current left feeder voltage.
     */
    double getLeftFeederVoltage();

    /**
     * Returns the current voltage being applied to the right feeder motor.
     * @return The current right feeder voltage.
     */
    double getRightFeederVoltage();

    /**
     * Returns the current RPS of the left feeder motor.
     * @return The current left feeder RPS.
     */
    double getLeftFeederRPS();

    /**
     * Returns the current RPS of the right feeder motor.
     * @return The current right feeder RPS.
     */
    double getRightFeederRPS();

    /**
     * Returns the target RPS for the feeder motors.
     * @return The target RPS.
     */
    double getTargetRPS();

    /**
     * Returns the target voltage for the feeder motors.
     * @return The target voltage.
     */
    double getTargetVoltage();

}