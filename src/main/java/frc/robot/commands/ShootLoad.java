package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootLoad extends Command {
  private final ShooterSubsystem shooters;
  private final double reserveSeconds;

  public ShootLoad(ShooterSubsystem shooters) {
    this(shooters, 5.5);
  }

  public ShootLoad(ShooterSubsystem shooters, double reserveSeconds) {
    this.shooters = shooters;
    this.reserveSeconds = reserveSeconds;
    addRequirements(shooters);
  }

  @Override
  public void initialize() {
    shooters.startBothShootersWasher();
  }

  @Override
  public boolean isFinished() {
    double t = DriverStation.getMatchTime();
    return t >= 0 && t <= reserveSeconds;
  }

  @Override
  public void end(boolean interrupted) {
    shooters.stopShootersWasher();
  }
}
