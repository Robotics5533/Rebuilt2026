package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RunRollers;

public class ThrustyTime extends Command {
    private final Command fullCommand;

    public ThrustyTime(IntakeSubsystem intake, RunRollers runRollers) {
        addRequirements(intake);
        addRequirements(runRollers);

        Command thrustAction = Commands.sequence(
            intake.flipManualReverseCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.8),
            intake.flipManualForwardCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.20),
             Commands.waitSeconds(1)
        ).withTimeout(3);

        fullCommand = runRollers.runRollerReverseCommand().alongWith(thrustAction);
        fullCommand.setName("ThrustyTime");
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