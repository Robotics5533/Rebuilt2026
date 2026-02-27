package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class ShootLoad extends Command {
  private final Command fullCommand;

  public ShootLoad(ShooterSubsystem shooters, WasherSubsystem washers, FeederSubsystem feeders) {
    this(shooters, washers, feeders, 5.0);
  }

  public ShootLoad(ShooterSubsystem shooters, WasherSubsystem washers, FeederSubsystem feeders,
      double durationSeconds) {
    addRequirements(shooters, washers, feeders);

    fullCommand = shooters.runInterpolatedShot(shooters::getHubDistance)
        .andThen(Commands.parallel(
            washers.run(Constants.ShooterConstants.washerVoltage),
            feeders.runBothFeedersCommand()))
        .withTimeout(durationSeconds)
        .finallyDo(interrupted -> {
          shooters.stopShooters();
          washers.stopWasher();
          feeders.stopFeeders();
        });
  }

  @Override
  public void initialize() {
    fullCommand.initialize();
  }

  @Override
  public void execute() {
    fullCommand.execute();
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
