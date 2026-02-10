package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WasherSubsystem;

public class ShootLoad extends Command {
  private final ShooterSubsystem shooters;
  private final WasherSubsystem washers;
  private final double durationSeconds;
  private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
  private boolean isShooting = false;

  public ShootLoad(ShooterSubsystem shooters, WasherSubsystem washers) {
    this(shooters, washers, 2.0);
  }

  public ShootLoad(ShooterSubsystem shooters, WasherSubsystem washers, double durationSeconds) {
    this.shooters = shooters;
    this.washers = washers;
    this.durationSeconds = durationSeconds;
    addRequirements(shooters, washers);
  }

  @Override
  public void initialize() {
    shooters.startBothShooters();
    timer.reset();
    timer.stop();
    isShooting = false;
  }

  @Override
  public void execute() {
    if (shooters.areShootersAtSpeed()) {
      if (!isShooting) {
        isShooting = true;
        timer.restart();
      }
      washers.runWasher(Constants.ShooterConstants.washerVoltage);
    } else {
      washers.stopWasher();
    }
  }

  @Override
  public boolean isFinished() {
    return isShooting && timer.hasElapsed(durationSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    shooters.stopShooters();
    washers.stopWasher();
  }
}
