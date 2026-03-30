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
import frc.robot.commands.AutoAutonAlignAndPass;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RunRollers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;


@Logged
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

    public void configureDriver(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, Superstructure superstructure) {
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
    }


    public void configureOperator(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake,
        ShooterSubsystem shooters, WasherSubsystem washers, FeederSubsystem feeders, Superstructure superstructure, LimelightSubsystem limelight, RunRollers runRollers) {

        // Left Trigger: Feed/Washer
        operator.leftTrigger().whileTrue(
            Commands.parallel(
                washers.run(Constants.ShooterConstants.washerVoltage),
                feeders.runBothFeedersCommand(),
                runRollers.runRollerReverseCommand())
            
                .finallyDo(interrupted -> {
                    washers.stopWasher();
                    feeders.stopFeeders();
                    runRollers.stopRoller();
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
            .onTrue(runRollers.runRollerReverseCommand())
            .onFalse(runRollers.runOnce(runRollers::stopRoller));

        // Right Bumper: Intake power in
        operator.leftBumper()
            .onTrue(runRollers.runRollerForwardCommand())
            .onFalse(runRollers.runOnce(runRollers::stopRoller));

        // B and X Buttons: Manual Intake Control

        operator.b().whileTrue(intake.flipManualForwardCommand(Constants.IntakeConstants.manualFlipVoltage));
        operator.x().whileTrue(intake.flipManualReverseCommand(Constants.IntakeConstants.manualFlipVoltage));

        operator.povRight()
                .onTrue(Commands.parallel(washers.run(-Constants.ShooterConstants.washerVoltage), feeders.InvertrunBothFeedersCommand(),
                        shooters.runFixedRPSShoot(-Constants.ShooterConstants.shooterVelocityRPS)))
                .onFalse(Commands.parallel(runRollers.runOnce(runRollers::stopRoller), washers.runOnce(washers::stopWasher),
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

        operator.povLeft().whileTrue(Commands.parallel(
            shooters.runFixedRPSShoot(Constants.ShooterConstants.shooterPassVelocityRPS),
            runRollers.runRollerForwardCommand(),
            feeders.runBothFeedersCommand(),
            washers.run(Constants.ShooterConstants.washerVoltage))
                .finallyDo(interrupted -> {
                    shooters.stopShooters();
                    shooters.resetRPSAdjustment(); // Reset adjustment after shooting
                    washers.stopWasher();
                    feeders.stopFeeders();
                    runRollers.stopRoller();
                })
                .withName("RunFixedRPSShoot"));
        // Original operator.povLeft() for AutoAlignAndShoot, if still desired.
        // operator.povLeft().and(new Trigger(() -> climb.inAllianceZone()))
        //     .whileTrue(
        //         new AutoAlignAndShoot(drivetrain, shooters, feeders, washers, climb, runRollers));


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