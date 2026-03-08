package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class FlipOutIntake extends Command {
    private final Command fullCommand;

    public FlipOutIntake(IntakeSubsystem intake, double durationSeconds) {
        addRequirements(intake);

        fullCommand = intake.flipManualForwardCommand(Constants.IntakeConstants.manualFlipVoltage)
            .withTimeout(durationSeconds);
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
