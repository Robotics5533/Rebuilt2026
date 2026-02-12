package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.utils.FieldPositions;
import frc.robot.utils.MathUtil;
import frc.robot.utils.Controls;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class RobotContainer {

        private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * (Constants.DriveConstants.SPEED_MULTIPLIER / 100.0);
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * (Constants.DriveConstants.SPEED_MULTIPLIER / 100.0);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * Constants.DriveConstants.DEADBAND)
                        .withRotationalDeadband(MaxAngularRate * Constants.DriveConstants.DEADBAND)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final Controls controls = new Controls();
        private final Field2d fieldViz = new Field2d();

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final LimelightSubsystem limelight = new LimelightSubsystem(Constants.LimelightConstants.LIMELIGHT_NAME,
                        drivetrain);
        private final ClimbSubsystem climb = new ClimbSubsystem();
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final ShooterSubsystem shooters = new ShooterSubsystem();
        private final WasherSubsystem washers = new WasherSubsystem();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                NamedCommands.registerCommand("shoot_load", new frc.robot.commands.ShootLoad(shooters, washers));
                NamedCommands.registerCommand("intake",
                                intake.runOnce(() -> intake.setFlip(IntakeSubsystem.FlipState.Out))
                                                .andThen(intake.run(intake::runRollerForward)));
                NamedCommands.registerCommand("stop_intake", intake.runOnce(intake::stopRoller)
                                .andThen(intake.runOnce(() -> intake.setFlip(IntakeSubsystem.FlipState.In))));

                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);
                SmartDashboard.putData("Field", fieldViz);

                configureBindings();
                FollowPathCommand.warmupCommand();
        }

        private void configureBindings() {

                drivetrain.setDefaultCommand(drivetrain.run(() -> {
                        drivetrain.setControl(
                                        drive.withVelocityX(controls.getDriveX() * MaxSpeed)
                                                        .withVelocityY(controls.getDriveY() * MaxSpeed)
                                                        .withRotationalRate(controls.getDriveOmega() * MaxAngularRate));

                        fieldViz.setRobotPose(drivetrain.getState().Pose);

                        fieldViz.getObject("BlueHub").setPose(FieldPositions.getBlueHubPose());
                        fieldViz.getObject("RedHub").setPose(FieldPositions.getRedHubPose());

                        fieldViz.getObject("BlueTowerRight")
                                        .setPose(FieldPositions.getBlueTowerRightPose());
                        fieldViz.getObject("RedTowerRight")
                                        .setPose(FieldPositions.getRedTowerRightPose());

                        fieldViz.getObject("BlueBumpLeft")
                                        .setPose(FieldPositions.getBlueBumpLeftPose());
                        fieldViz.getObject("BlueBumpRight")
                                        .setPose(FieldPositions.getBlueBumpRightPose());
                        fieldViz.getObject("RedBumpLeft")
                                        .setPose(FieldPositions.getRedBumpLeftPose());
                        fieldViz.getObject("RedBumpRight")
                                        .setPose(FieldPositions.getRedBumpRightPose());

                        updateDynamicObstacles();
                }));

                controls.configureDriver(drivetrain, limelight);
                controls.configureOperator(drivetrain, climb, intake, shooters, washers, limelight);

                // climb.setDefaultCommand(climb.run(climb::applySetpoint).ignoringDisable(true));

                // controls.getDriver().y().onTrue(pathfindToRightTower());

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                RobotModeTriggers.disabled()
                                .onTrue(drivetrain.runOnce(() -> drivetrain.setNeutralMode(NeutralModeValue.Coast)));
                RobotModeTriggers.disabled().negate()
                                .onTrue(drivetrain.runOnce(() -> drivetrain.setNeutralMode(NeutralModeValue.Brake)));

                new Trigger(shooters::areShootersAtSpeed)
                                .onTrue(Commands.runOnce(() -> controls.setOperatorRumble(0.5))
                                                .andThen(Commands.waitSeconds(0.2))
                                                .andThen(Commands.runOnce(() -> controls.setOperatorRumble(0))));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void updateDynamicObstacles() {
                List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();

                obstacles.add(MathUtil.createBoundingBox(
                                Constants.FieldConstants.blueBumpLeftPose.getTranslation(),
                                Constants.FieldConstants.bumpWidth, Constants.FieldConstants.bumpDepth));

                obstacles.add(MathUtil.createBoundingBox(
                                Constants.FieldConstants.blueBumpRightPose.getTranslation(),
                                Constants.FieldConstants.bumpWidth, Constants.FieldConstants.bumpDepth));

                obstacles.add(MathUtil.createBoundingBox(
                                Constants.FieldConstants.redBumpLeftPose.getTranslation(),
                                Constants.FieldConstants.bumpWidth, Constants.FieldConstants.bumpDepth));

                obstacles.add(MathUtil.createBoundingBox(
                                Constants.FieldConstants.redBumpRightPose.getTranslation(),
                                Constants.FieldConstants.bumpWidth, Constants.FieldConstants.bumpDepth));

                Pathfinding.setDynamicObstacles(
                                obstacles,
                                drivetrain.getState().Pose.getTranslation());
        }

        public Command pathfindToRightTower() {

                var target = frc.robot.utils.AllianceUtil.isRedAlliance()
                                ? FieldPositions.getRedTowerRightPose()
                                : FieldPositions.getBlueTowerRightPose();

                PathConstraints constraints = new PathConstraints(
                                MaxSpeed * 0.8,
                                MaxSpeed * 1.2,
                                MaxAngularRate * 0.8,
                                MaxAngularRate * 1.2);

                return AutoBuilder.pathfindToPose(target, constraints, 0.0);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
