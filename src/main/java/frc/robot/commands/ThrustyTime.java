package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.depoIntakeSubsystem;

public class ThrustyTime extends Command {
    private final Command fullCommand;

    public ThrustyTime(IntakeSubsystem intake, depoIntakeSubsystem depoIntake, int loopCount) {
        addRequirements(intake);
        addRequirements(depoIntake);

        Command[] commands = new Command[loopCount * 3];
        for (int i = 0; i < loopCount; i++) {
            commands[i * 3] = intake.flipManualReverseCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.8);
            commands[i * 3 + 1] = intake.flipManualForwardCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.25);
            commands[i * 3 + 2] = Commands.waitSeconds(1);
        }
        
        fullCommand = depoIntake.runRollerReverseCommand().alongWith(Commands.sequence(commands));
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
