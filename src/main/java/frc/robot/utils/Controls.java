package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.AutoAlignAndShoot;
import frc.robot.commands.AutoAlignCommand; // New import
import frc.robot.commands.ShakeIntake;
import frc.robot.commands.ShootAtDistance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.RunRollers;
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
            new AutoAlignCommand(drivetrain, () -> Rotation2d.fromDegrees(AllianceUtil.getTargetHeadingToHub(drivetrain, Constants.LimelightConstants.LIMELIGHT_NAME)), () -> getDriveX(), () -> getDriveY(), superstructure::setAligned)); 

        driver.leftBumper().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.a().whileTrue(
            drivetrain.applyRequest(
                () -> new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake()));

        driver.b().whileTrue(drivetrain.applyRequest(
            () -> new com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt().withModuleDirection(
                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        driver.y().whileTrue(new AutoAlignCommand(drivetrain, () -> Rotation2d.kZero, () -> 0.0, () -> 0.0)); // Updated for facing 0 degrees with no translation

        // Climb controls
        if (climb != null) {
            driver.povUp().whileTrue(climb.runManualClimbCommand(Constants.ClimbConstants.maxVoltage));
            driver.povDown().whileTrue(climb.runManualClimbCommand(-Constants.ClimbConstants.maxVoltage));
        }
    }

    public void configureOperator(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake,
        ShooterSubsystem shooters, WasherSubsystem washers, FeederSubsystem feeders, Superstructure superstructure, ClimbSubsystem climb, LimelightSubsystem limelight ) {

        // Left Trigger: Feed/Washer
        operator.leftTrigger().whileTrue(
            Commands.parallel(
                washers.run(Constants.ShooterConstants.washerVoltage),
                feeders.runBothFeedersCommand(),
                new ShakeIntake(intake))
            
                .finallyDo(interrupted -> {
                    washers.stopWasher();
                    feeders.stopFeeders();
                    intake.stopRoller();
                    intake.stopFlip();
                })
                .withName("RunWashersAndFeedersAndShake"));

        // Right Trigger: Shooting (Interpolated)
        operator.rightTrigger().whileTrue(
            shooters.runInterpolatedShot(shooters::getHubDistance)
                .finallyDo(interrupted -> {
                    shooters.stopShooters();
                    shooters.resetRPSAdjustment();
                })
                .withName("RunBothShooters"));

        // Left Bumper: Intake power out
        operator.rightBumper()
            .onTrue(intake.run(intake::runRollerReverse))
            .onFalse(intake.runOnce(intake::stopRoller));

        // Right Bumper: Intake power in
        operator.leftBumper()
            .onTrue(intake.run(intake::runRollerForward))
            .onFalse(intake.runOnce(intake::stopRoller));

        // B and X Buttons: Manual Intake Control

        operator.b().whileTrue(intake.flipManualForwardCommand(Constants.IntakeConstants.manualFlipVoltage));
        operator.x().whileTrue(intake.flipManualReverseCommand(Constants.IntakeConstants.manualFlipVoltage));

        operator.povRight()
                .onTrue(Commands.parallel(washers.run(-Constants.ShooterConstants.washerVoltage), feeders.InvertrunBothFeedersCommand(),
                        shooters.runFixedRPSShoot(-Constants.ShooterConstants.shooterVelocityRPS)))
                .onFalse(Commands.parallel(intake.runOnce(intake::stopRoller), washers.runOnce(washers::stopWasher),
                        feeders.runOnce(feeders::stopFeeders), shooters.runOnce(shooters::stopShooters)));
        // Y Button: Shooter with fixed RPS
        operator.y().whileTrue(
            shooters.runFixedRPSShoot(Constants.ShooterConstants.shooterVelocityRPS)
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

        // Operator A Button: Shoot at Distance
        operator.a().whileTrue(new ShootAtDistance(shooters, washers, feeders));


        // Original operator.povLeft() for AutoAlignAndShoot, if still desired.
        operator.povLeft().and(new Trigger(() -> superstructure.inAllianceZone()))
            .whileTrue(
                new AutoAlignAndShoot(drivetrain, shooters, feeders, washers, superstructure, limelight));


    }

    public void setOperatorRumble(double intensity) {
        operator.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, intensity);
    }

    public void setDriverRumble(double intensity) {
        driver.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, intensity);
    }

    public CommandXboxController getDriver() {
        return driver;
    }

    public CommandXboxController getOperator() {
        return operator;
    }
}