package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RunRollers;

public class RunIntakeRollersDouble extends Command {
    private final Command fullCommand;

    public RunIntakeRollersDouble(RunRollers runRollers, double durationSeconds) {
        addRequirements(runRollers);

        fullCommand = (runRollers.runRollerReverseCommand().withTimeout(durationSeconds));
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
