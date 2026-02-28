package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.AutoAlignAndShoot;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ShootAtDistance;
import frc.robot.Constants.IntakeFlipConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeFlip;
import frc.robot.subsystems.IntakeRoller;
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

    public void configureDriver(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight,
            Superstructure superstructure, ClimbSubsystem climb) {

        driver.rightBumper().and(superstructure.activeHubTrigger)
                .and(new Trigger(() -> superstructure.inAllianceZone())).whileTrue(
                        new AutoAlignCommand(drivetrain,
                                () -> Rotation2d.fromDegrees(AllianceUtil.getTargetHeadingToHub(drivetrain,
                                        Constants.LimelightConstants.LIMELIGHT_NAME)),
                                () -> getDriveX(), () -> getDriveY())); 

        driver.leftBumper().onTrue(
                drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.a().whileTrue(
                drivetrain.applyRequest(
                        () -> new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake()));

        driver.b().whileTrue(drivetrain.applyRequest(
                () -> new com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt().withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        driver.y().whileTrue(new AutoAlignCommand(drivetrain, () -> Rotation2d.kZero, () -> 0.0, () -> 0.0)); 
                                                                                                              
                                                                                                              
                                                                                                              
                                                                                                              
                                                                                                              
                                                                                                              

        
        if (climb != null) {
            driver.povUp().whileTrue(climb.runManualClimbCommand(Constants.ClimbConstants.maxVoltage));
            driver.povDown().whileTrue(climb.runManualClimbCommand(-Constants.ClimbConstants.maxVoltage));
        }
    }

    public void configureOperator(CommandSwerveDrivetrain drivetrain, IntakeRoller intakeRoller, IntakeFlip intakeFlip,
            ShooterSubsystem shooters, WasherSubsystem washers, FeederSubsystem feeders, Superstructure superstructure,
            ClimbSubsystem climb, LimelightSubsystem limelight) {

        
        operator.leftTrigger().whileTrue(
                Commands.parallel(
                        washers.run(Constants.ShooterConstants.washerVoltage),
                        feeders.runBothFeedersCommand())
                        .finallyDo(interrupted -> {
                            washers.stopWasher();
                            feeders.stopFeeders();
                        })
                        .withName("RunWashersAndFeeders"));

        
        operator.rightTrigger().whileTrue(
                shooters.runInterpolatedShot(shooters::getHubDistance)
                        .finallyDo(interrupted -> {
                            shooters.stopShooters();
                            shooters.resetRPSAdjustment();
                        })
                        .withName("RunBothShooters"));

        
        operator.rightBumper()
                .onTrue(Commands.run(() -> intakeRoller.runRollerReverse(), intakeRoller))
                .onFalse(Commands.runOnce(() -> intakeRoller.stopRoller(), intakeRoller));

        
        operator.leftBumper()
                .onTrue(Commands.run(() -> intakeRoller.runRollerForward(), intakeRoller))
                .onFalse(Commands.runOnce(() -> intakeRoller.stopRoller(), intakeRoller));

        
 
        operator.b().onTrue(intakeFlip.outPosition());

      
        operator.x()
            .whileTrue(Commands.run(() -> intakeFlip.runManual(IntakeFlipConstants.manualFlipVoltage), intakeFlip))
            .onFalse(Commands.runOnce(intakeFlip::stopManual, intakeFlip)
            .andThen(Commands.either(intakeFlip.inPosition(), intakeFlip.outPosition(), intakeFlip::isAtInPosition)));

        
        operator.y().whileTrue(
                shooters.runFixedRPSShoot(Constants.ShooterConstants.shooterVelocityRPS)
                        .finallyDo(interrupted -> {
                            shooters.stopShooters();
                            shooters.resetRPSAdjustment(); 
                        })
                        .withName("RunFixedRPSShoot"));

        
        operator.povUp().onTrue(
                shooters.runOnce(() -> shooters.incrementRPSAdjustment(Constants.ShooterConstants.rpsAdjustmentDelta)));

        
        operator.povDown().onTrue(
                shooters.runOnce(
                        () -> shooters.incrementRPSAdjustment(-Constants.ShooterConstants.rpsAdjustmentDelta)));

        
        operator.a().whileTrue(new ShootAtDistance(shooters, washers, feeders));

        
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