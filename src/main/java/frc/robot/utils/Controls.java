package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.AutoAlignHub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
 import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;

public class Controls {
        private final CommandXboxController driver;
        private final CommandXboxController operator;

        public Controls() {
                driver = new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
                operator = new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);
        }

        public double getDriveX() {
                return -MathUtil.applyDeadband(driver.getLeftY(), Constants.DriveConstants.DEADBAND);
        }

        public double getDriveY() {
                return -MathUtil.applyDeadband(driver.getLeftX(), Constants.DriveConstants.DEADBAND);
        }

        public double getDriveOmega() {
                return -MathUtil.applyDeadband(driver.getRightX(), Constants.DriveConstants.DEADBAND);
        }

        public void configureDriver(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
                driver.rightBumper().whileTrue(
                                new AutoAlignHub(drivetrain, limelight, driver));

                driver.leftBumper().onTrue(
                                drivetrain.runOnce(drivetrain::seedFieldCentric));

                driver.a().whileTrue(
                                drivetrain.applyRequest(
                                                () -> new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake()));

                driver.b().whileTrue(drivetrain.applyRequest(
                                () -> new com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt().withModuleDirection(
                                                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
        }

        public void configureOperator(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake,
                        ShooterSubsystem shooters, WasherSubsystem washers, FeederSubsystem feeders) {

                operator.a().onTrue(intake.runOnce(intake::toggleFlip)
                                .andThen(Commands.runOnce(() -> setOperatorRumble(0.5)))
                                .andThen(Commands.waitSeconds(0.2))
                                .andThen(Commands.runOnce(() -> setOperatorRumble(0))));

                operator.rightBumper()
                                .onTrue(intake.run(intake::runRollerForward))
                                .onFalse(intake.runOnce(intake::stopRoller));

                operator.leftBumper()
                                .onTrue(intake.run(intake::runRollerReverse))
                                .onFalse(intake.runOnce(intake::stopRoller));

                operator.leftTrigger(Constants.OperatorConstants.TRIGGER_THRESHOLD)
                                .onTrue(
                                                Commands.parallel(
                                                                washers.run(Constants.ShooterConstants.washerVoltage),
                                                                feeders.runBothFeedersCommand()))
                                .onFalse(
                                                Commands.parallel(
                                                                washers.runOnce(washers::stopWasher),
                                                                feeders.stopFeedersCommand()));

                operator.rightTrigger(Constants.OperatorConstants.TRIGGER_THRESHOLD)
                                .onTrue(
                                                shooters.runBothShootersToSpeedCommand())
                                .onFalse(
                                                shooters.stopShootersCommand());

                operator.x().whileTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> setOperatorRumble(0.5)),
                                                Commands.waitSeconds(0.2),
                                                Commands.runOnce(() -> setOperatorRumble(0)),
                                                intake.flipManualForwardCommand(3.0)
                                                                .withInterruptBehavior(
                                                                                edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming)));
                operator.back().whileTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> setOperatorRumble(0.5)),
                                                Commands.waitSeconds(0.2),
                                                Commands.runOnce(() -> setOperatorRumble(0)),
                                                intake.flipManualReverseCommand(3.0)
                                                                .withInterruptBehavior(
                                                                                edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming)));
        }

        public void setOperatorRumble(double intensity) {
                operator.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, intensity);
        }

        public CommandXboxController getDriver() {
                return driver;
        }

        public CommandXboxController getOperator() {
                return operator;
        }
}
