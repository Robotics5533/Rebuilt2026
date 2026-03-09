package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ShakeIntake extends Command {
    private final Command fullCommand;

    public ShakeIntake(IntakeSubsystem intake) {
        addRequirements(intake);

        Command shakeAction = Commands.sequence(
            intake.flipManualReverseCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.8),
            intake.flipManualForwardCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.25)
        ).repeatedly();

        fullCommand = intake.runRollerReverseCommand().alongWith(shakeAction);
        fullCommand.setName("ShakeIntake");
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
