package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Represents the robot's washer mechanism, typically used to prepare game pieces
 * for shooting or to clear the shooting path. It controls two Falcon 500 (TalonFX) motors.
 */
public class WasherSubsystem extends SubsystemBase {
    // TalonFX motor controllers for the left and right washer motors.
    private final TalonFX leftWasher = new TalonFX(Constants.ShooterConstants.leftWasherMotorId);
    private final TalonFX rightWasher = new TalonFX(Constants.ShooterConstants.rightWasherMotorId);

    // Control request for the TalonFXs, used for voltage control.
    private final VoltageOut voltageCtrl = new VoltageOut(0);

    /**
     * Constructs a new WasherSubsystem.
     * Configures the TalonFX motor controllers with a neutral mode of Brake.
     */
    public WasherSubsystem() {
        // Set the neutral mode to Brake, causing the motors to actively resist movement when idle.
        leftWasher.setNeutralMode(NeutralModeValue.Brake);
        rightWasher.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Runs both washer motors at a specified voltage.
     * The right washer motor spins in the opposite direction of the left washer motor.
     * @param volts The voltage to apply to the washer motors.
     */
    public void runWasher(double volts) {
        leftWasher.setControl(voltageCtrl.withOutput(volts));
        rightWasher.setControl(voltageCtrl.withOutput(-volts));
    }

    /**
     * Stops both washer motors by setting their voltage to zero.
     */
    public void stopWasher() {
        leftWasher.setControl(voltageCtrl.withOutput(0));
        rightWasher.setControl(voltageCtrl.withOutput(0));
    }

    /**
     * Returns a command that runs the washer at a specified voltage.
     * The command will run continuously until interrupted and will stop the washer when it ends.
     * @param volts The voltage to apply to the washer motors.
     * @return A command to run the washer.
     */
    public Command run(double volts) {
        return run(() -> runWasher(volts)).finallyDo(this::stopWasher);
    }
}
