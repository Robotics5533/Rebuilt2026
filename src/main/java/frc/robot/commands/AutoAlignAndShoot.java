package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * A complex command that combines automatic alignment to the hub with a distance-based shooting sequence.
 * This command first aligns the robot to the hub and spins up the shooters to the calculated speed in parallel.
 * Once both conditions (alignment and shooter at speed) are met, it activates the feeder and washer subsystems
 * to initiate shooting. All mechanisms stop when the command is interrupted.
 */
public class AutoAlignAndShoot extends Command {
    private final Command fullCommand;
    private final AutoAlignHub autoAlignCommand;
    private final ShooterSubsystem shooter;

    /**
     * Creates a new AutoAlignAndShoot command.
     *
     * @param drivetrain The {@link CommandSwerveDrivetrain} subsystem for robot movement and alignment.
     * @param shooter The {@link ShooterSubsystem} for controlling shooter motors.
     * @param feeder The {@link FeederSubsystem} for controlling the feeder mechanism.
     * @param washer The {@link WasherSubsystem} for controlling the washer mechanism.
     * @param superstructure The {@link Superstructure} for alliance zone checks.
     * @param limelight The {@link LimelightSubsystem} for vision data used in alignment.
     */
    public AutoAlignAndShoot(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            WasherSubsystem washer,
            Superstructure superstructure,
            LimelightSubsystem limelight) {

        // Initialize the AutoAlignHub command, which handles the rotational alignment.
        // The controller parameter is null because this command manages the drivetrain's movement.
        autoAlignCommand = new AutoAlignHub(drivetrain, limelight, null);
        this.shooter = shooter;

        // Declare all required subsystems to prevent conflicts and ensure exclusive access.
        addRequirements(drivetrain, shooter, feeder, washer, superstructure, limelight);

        // Define the full sequence of actions for this command.
        fullCommand = Commands.parallel(
                // 1. Run the auto-alignment command in parallel with other actions.
                autoAlignCommand,
                // 2. Spin up the shooters to the interpolated speed based on distance to the hub.
                // This runs continuously while the command is active.
                shooter.runInterpolatedShot(shooter::getHubDistance),
                // 3. Define a sequential action that waits for conditions before activating feeders/washer.
                Commands.sequence(
                    // Wait until both the robot is aligned AND the shooters are at target speed.
                    Commands.waitUntil(() ->
                        autoAlignCommand.isAligned() &&
                        shooter.areShootersAtSpeed()
                    ),
                    // Once conditions are met, run the feeder and washer in parallel.
                    // These will continue to run until the overall command is interrupted.
                    Commands.parallel(
                        feeder.runBothFeedersCommand(),
                        washer.run(Constants.ShooterConstants.washerVoltage)
                    )
                )
            )
            // Specify behavior when the command is interrupted: ensure all mechanisms stop.
            .finallyDo(interrupted -> {
                drivetrain.setControl(new SwerveRequest.SwerveDriveBrake()); // Brake the drivetrain.
                shooter.stopShooters(); // Stop shooter motors.
                feeder.stopFeeders();   // Stop feeder motors.
                washer.stopWasher();   // Stop washer motor.
            })
            .withName("AutoAlignAndShoot"); // Assign a descriptive name for debugging and logging.
    }

    @Override
    public void initialize() {
        fullCommand.initialize();
    }

    @Override
    public void execute() {
        fullCommand.execute();
        // Log key states for debugging and analysis
        SmartDashboard.putBoolean("AutoAlignAndShoot/Aligned", autoAlignCommand.isAligned());
        SmartDashboard.putBoolean("AutoAlignAndShoot/ShootersAtSpeed", shooter.areShootersAtSpeed());
        SmartDashboard.putNumber("AutoAlignAndShoot/HubDistance", shooter.getHubDistance());
    }

    @Override
    public boolean isFinished() {
        return fullCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        fullCommand.end(interrupted);
    }
}
