package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.epilogue.Logged;
//import edu.wpi.first.epilogue.Epilogue;

/**
 * A command to perform a shot sequence based on the robot's distance to the speaker,
 * as reported by the Limelight. It runs the shooters up to speed, then runs the
 * washer and feeders for a set duration, and finally stops all components.
 */
//@Logged
public class ShootAtDistance extends Command {
  private final Command fullCommand;

  /**
   * Creates a new ShootAtDistance command.
   * This command will run the shooters to speed based on the hub distance,
   * then run the washer and feeders until the command is interrupted (e.g., button released).
   * @param shooters The ShooterSubsystem instance.
   * @param washers The WasherSubsystem instance.
   * @param feeders The FeederSubsystem instance.
   */
  public ShootAtDistance(ShooterSubsystem shooters, WasherSubsystem washers, FeederSubsystem feeders) {
    addRequirements(shooters, washers, feeders);

    fullCommand = Commands.parallel(
        shooters.runInterpolatedShot(shooters::getHubDistance),
        Commands.sequence(
            Commands.waitUntil(shooters::areShootersAtSpeed),
            Commands.parallel(
                washers.run(Constants.ShooterConstants.washerVoltage),
                feeders.runBothFeedersCommand()
            )
        )
    )
    .finallyDo(interrupted -> {
      shooters.stopShooters();
      shooters.resetRPSAdjustment();
      washers.stopWasher();
      feeders.stopFeeders();
    })
    .withName("ShootAtDistance");
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
