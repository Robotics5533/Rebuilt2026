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
public class ManualIntakeFlip extends SubsystemBase {
    // TalonFX motor controllers for the left and right washer motors.
    private final TalonFX Intake = new TalonFX(Constants.IntakeConstants.intakeFlipMotorId);


    // Control request for the TalonFXs, used for voltage control.
    private final VoltageOut voltageCtrl = new VoltageOut(0);

    /**
     * Constructs a new WasherSubsystem.
     * Configures the TalonFX motor controllers with a neutral mode of Brake.
     */
    public ManualIntakeFlip() {
        // Set the neutral mode to Brake, causing the motors to actively resist movement when idle.
       Intake.setNeutralMode(NeutralModeValue.Brake);
     
    }

    /**
     * Runs both washer motors at a specified voltage.
     * The right washer motor spins in the opposite direction of the left washer motor.
     * @param volts The voltage to apply to the washer motors.
     */
    public void runIntakeUp(double volts) {
        Intake.setControl(voltageCtrl.withOutput(10));
    }
     public void runIntakeDown(double volts) {
        Intake.setControl(voltageCtrl.withOutput(-10));
    }

    /**
     * Stops both washer motors by setting their voltage to zero.
     */
    public void stopIntake() {
        Intake.setControl(voltageCtrl.withOutput(0));
    }
      

    /**
     * Returns a command that runs the washer at a specified voltage.
     * The command will run continuously until interrupted and will stop the washer when it ends.
     * @param volts The voltage to apply to the washer motors.
     * @return A command to run the washer.
     */
    public Command run(double volts) {
        return run(() -> runIntakeUp(volts)).finallyDo(this::stopIntake);
    }
    public Command runDown(double volts) {
        return run(() -> runIntakeDown(volts)).finallyDo(this::stopIntake);
    }
}
