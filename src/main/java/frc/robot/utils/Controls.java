package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.AutoAlignAndShoot;
import frc.robot.commands.AutoAlignHub;
import frc.robot.commands.FaceAngle; // New import

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClimbSubsystem;import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
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

    public void configureDriver(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, Superstructure superstructure, ClimbSubsystem climb) {
        driver.rightBumper().and(superstructure.activeHubTrigger).and(new Trigger(() -> superstructure.inAllianceZone())).whileTrue(
            new AutoAlignHub(drivetrain, limelight, driver));

        driver.leftBumper().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.a().whileTrue(
            drivetrain.applyRequest(
                () -> new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake()));

        driver.b().whileTrue(drivetrain.applyRequest(
            () -> new com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt().withModuleDirection(
                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // Driver Y Button: Face 0 degrees
        driver.y().whileTrue(new FaceAngle(drivetrain, Rotation2d.kZero));

        // Climb controls
        if (climb != null) {
            driver.povUp().whileTrue(climb.runManualClimbCommand(Constants.ClimbConstants.maxVoltage));
            driver.povDown().whileTrue(climb.runManualClimbCommand(-Constants.ClimbConstants.maxVoltage));
        }
    }

    public void configureOperator(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake,
        ShooterSubsystem shooters, WasherSubsystem washers, FeederSubsystem feeders, Superstructure superstructure, ClimbSubsystem climb, LimelightSubsystem limelight) {

        // Left Trigger: Feed/Washer
        operator.leftTrigger().whileTrue(
            Commands.parallel(
                washers.run(Constants.ShooterConstants.washerVoltage),
                feeders.runBothFeedersCommand())
                .finallyDo(interrupted -> {
                    washers.stopWasher();
                    feeders.stopFeeders();
                })
                .withName("RunWashersAndFeeders"));

        // Right Trigger: Shooting (Interpolated)
        operator.rightTrigger().whileTrue(
            shooters.runInterpolatedShot(shooters::getHubDistance)
                .finallyDo(interrupted -> {
                    shooters.stopShooters();
                    shooters.resetRPSAdjustment(); // Reset adjustment after shooting
                })
                .withName("RunBothShooters"));

        // Left Bumper: Intake power out
        operator.leftBumper()
            .onTrue(intake.run(intake::runRollerReverse))
            .onFalse(intake.runOnce(intake::stopRoller));

        // Right Bumper: Intake power in
        operator.rightBumper()
            .onTrue(intake.run(intake::runRollerForward))
            .onFalse(intake.runOnce(intake::stopRoller));

        // B Button: Intake out position
        operator.b().onTrue(intake.runOnce(() -> intake.setFlip(IntakeSubsystem.FlipState.Out)));

        // X Button: Intake in position
        operator.x().onTrue(intake.stowIntake());

        // Y Button: Shooter with fixed RPS (47.5)
        operator.y().whileTrue(
            shooters.runFixedRPSShoot(47.5)
                .finallyDo(interrupted -> {
                    shooters.stopShooters();
                    shooters.resetRPSAdjustment(); // Reset adjustment after shooting
                })
                .withName("RunFixedRPSShoot"));

        // Operator D-pad Up: Increment Shooter RPS Adjustment
        operator.povUp().onTrue(
            shooters.runOnce(() -> shooters.incrementRPSAdjustment(Constants.ShooterConstants.rpsAdjustmentDelta)));

        // Operator D-pad Down: Decrement Shooter RPS Adjustment
        operator.povDown().onTrue(
            shooters.runOnce(() -> shooters.incrementRPSAdjustment(-Constants.ShooterConstants.rpsAdjustmentDelta)));


        // Original operator.povLeft() for AutoAlignAndShoot, if still desired.
        operator.povLeft().and(new Trigger(() -> superstructure.inAllianceZone()))
            .whileTrue(
                new AutoAlignAndShoot(drivetrain, shooters, feeders, washers, superstructure, limelight));


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