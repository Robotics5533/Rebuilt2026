package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RunRollers;
import edu.wpi.first.epilogue.Logged;
//import edu.wpi.first.epilogue.Epilogue;

//@Logged
public class ThrustyTimeDepo extends Command {
    private final Command fullCommand;

    public ThrustyTimeDepo(IntakeSubsystem intake, RunRollers runRollers, int loopCount) {
        addRequirements(intake);
        addRequirements(runRollers);

        Command[] commands = new Command[loopCount * 3];
        for (int i = 0; i < loopCount; i++) {
            commands[i * 3] = intake.flipManualReverseCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.8);
            commands[i * 3 + 1] = intake.flipManualForwardCommand(Constants.IntakeConstants.manualFlipVoltage).withTimeout(0.25);
            commands[i * 3 + 2] = Commands.waitSeconds(1);
        }

        Command thrustAction = Commands.sequence(commands);
        fullCommand = runRollers.runRollerReverseCommand()
            .alongWith(thrustAction)
            .withTimeout(loopCount * 1.0)
            .finallyDo(stage -> runRollers.stopRoller());
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