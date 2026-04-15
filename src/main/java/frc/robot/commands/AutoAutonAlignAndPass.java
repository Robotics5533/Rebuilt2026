package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utils.AllianceUtil;

/**
 * A complex command that combines automatic alignment to the closest pass position
 * with a distance-based shooting sequence.
 */
public class AutoAutonAlignAndPass extends Command {
    private final Command fullCommand;
    private final AutoAlignCommand autoAlignCommand;
    private final ShooterSubsystem shooter;

    public AutoAutonAlignAndPass(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            WasherSubsystem washer,
            Superstructure superstructure,
            Limelight limelight) {

        autoAlignCommand = new AutoAlignCommand(drivetrain,
                () -> Rotation2d.fromDegrees(
                        AllianceUtil.getTargetHeadingToClosestPass(drivetrain)),
                () -> 0.0, () -> 0.0);
        this.shooter = shooter;

        addRequirements(drivetrain, shooter, feeder, washer, superstructure, limelight);

        fullCommand = Commands.parallel(
                autoAlignCommand,
                shooter.runInterpolatedPassShot(shooter::getPassDistance),
                Commands.sequence(
                        Commands.waitUntil(shooter::areShootersAtSpeed),
                        Commands.parallel(
                            washer.run(Constants.ShooterConstants.autonWasherVoltage),
                                feeder.autonrunBothFeedersCommand()
                                )))
                .finallyDo(interrupted -> {
                    shooter.stopShooters();
                    feeder.stopFeeders();
                    washer.stopWasher();
                })
                .withName("AutoAlignAndPass");
    }
    
    @Override
    public void initialize() {
        fullCommand.initialize();
    }

    @Override
    public void execute() {
        fullCommand.execute();

        SmartDashboard.putBoolean("AutoAlignAndPass/ShootersAtSpeed", shooter.areShootersAtSpeed());
        SmartDashboard.putNumber("AutoAlignAndPass/PassDistance", shooter.getPassDistance());
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