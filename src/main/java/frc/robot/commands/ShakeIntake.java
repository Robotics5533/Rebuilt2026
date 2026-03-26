package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RunRollers;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;

@Logged
public class ShakeIntake extends Command {
    private final Command fullCommand;

    public ShakeIntake(IntakeSubsystem intake, RunRollers runRollers) {
        addRequirements(intake);
        addRequirements(runRollers);

        Command shakeAction = Commands.sequence(
            Commands.waitSeconds(1),
            intake.flipManualReverseCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.35),
            Commands.waitSeconds(0.5),
            intake.flipManualForwardCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.075)
        ).repeatedly();

        fullCommand = runRollers.runRollerReverseCommand().alongWith(shakeAction);
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