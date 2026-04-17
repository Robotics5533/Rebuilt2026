package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
//import edu.wpi.first.epilogue.Epilogue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import frc.robot.commands.ThrustyTime;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAutonAlignAndShoot;
import frc.robot.commands.FlipOutIntake;
import frc.robot.commands.RunIntakeRollers;
import frc.robot.commands.RunIntakeRollersDouble;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.RunRollers;
import frc.robot.utils.Controls;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
//@Logged
public class RobotContainer {
        private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                        * (Constants.DriveConstants.SPEED_MULTIPLIER / 100.0);
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond)
                        * (Constants.DriveConstants.SPEED_MULTIPLIER / 100.0);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * Constants.DriveConstants.DEADBAND)
                        .withRotationalDeadband(MaxAngularRate * Constants.DriveConstants.DEADBAND)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final Controls controls = new Controls();

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final LimelightSubsystem limelight = new LimelightSubsystem(Constants.LimelightConstants.LIMELIGHT_NAME,
                        drivetrain);
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final ShooterSubsystem shooters = new ShooterSubsystem(drivetrain,
                        Constants.LimelightConstants.LIMELIGHT_NAME);
        private final RunRollers runRollers = new RunRollers();
        private final WasherSubsystem washers = new WasherSubsystem();
        private final FeederSubsystem feeder = new FeederSubsystem();
        private final Superstructure superstructure = new Superstructure(() -> drivetrain.getState().Pose);
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                LimelightHelpers.setCameraPose_RobotSpace(
                                Constants.LimelightConstants.LIMELIGHT_NAME,
                                0.0381,//0.381
                                0,//0.3175
                                0.5207,//0.079375
                                0.0,
                                7.5,
                                0.0);
                AutoAutonAlignAndShoot autoAutonAlignAndShootCommand = new AutoAutonAlignAndShoot(drivetrain, shooters, feeder, washers, superstructure, limelight, 2);
                AutoAutonAlignAndShoot depoautoAutonAlignAndShootCommand = new AutoAutonAlignAndShoot(drivetrain, shooters, feeder, washers, superstructure, limelight, 6);
                 AutoAutonAlignAndShoot backtomidautoAutonAlignAndShootCommand = new AutoAutonAlignAndShoot(drivetrain, shooters, feeder, washers, superstructure, limelight, 7);
                //  AutoAutonAlignAndPass autoAutonAlignAndPassCommand = new AutoAutonAlignAndPass(drivetrain, shooters, feeder, washers, superstructure, limelight, 3.0);
                
                

                NamedCommands.registerCommand("shoot_load",
                                new frc.robot.commands.ShootLoad(shooters, washers, feeder,5.0));

                NamedCommands.registerCommand("shoot_interpolated", autoAutonAlignAndShootCommand);

                NamedCommands.registerCommand("shoot_interpolatedbacktomid", backtomidautoAutonAlignAndShootCommand);

                NamedCommands.registerCommand("shoot_interpolateddepo", depoautoAutonAlignAndShootCommand);

                NamedCommands.registerCommand("flip_out_intake", intake.runOnce(intake::outFlip));

                NamedCommands.registerCommand("run_intake_rollers", new RunIntakeRollers(runRollers,4));
                
                NamedCommands.registerCommand("run_intake_rollers_Second", new RunIntakeRollersDouble(runRollers,1.29));

                NamedCommands.registerCommand("run_intake_rollers_Justin", new RunIntakeRollersDouble(runRollers,1.6));

                NamedCommands.registerCommand("deporoller", new RunIntakeRollers(runRollers,5));

                NamedCommands.registerCommand("face_otherside", new frc.robot.commands.AutoFace(drivetrain, shooters, feeder, washers, superstructure, limelight,0.25));

                NamedCommands.registerCommand("erect", intake.runOnce(intake::erect));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();

                CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        }

        /**
         * Use this method to define your button->command mappings.
         */
        private void configureBindings() {

                drivetrain.setDefaultCommand(drivetrain.run(() -> {
                        drivetrain.setControl(
                                        drive.withVelocityX(controls.getDriveX() * MaxSpeed)
                                                        .withVelocityY(controls.getDriveY() * MaxSpeed)
                                                        .withRotationalRate(controls.getDriveOmega() * MaxAngularRate));

                }));

                controls.configureDriver(drivetrain, limelight, superstructure);
                controls.configureOperator(drivetrain, intake, shooters, washers, feeder, superstructure, limelight, runRollers);

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                RobotModeTriggers.disabled()
                                .onTrue(drivetrain.runOnce(() -> drivetrain.setNeutralMode(NeutralModeValue.Coast)));
               RobotModeTriggers.disabled().negate()
    .onTrue(Commands.runOnce(() -> drivetrain.setNeutralMode(NeutralModeValue.Brake)));

                new Trigger(shooters::areShootersAtSpeed)
                                .onTrue(Commands.runOnce(() -> controls.setOperatorRumble(1))
                                                .andThen(Commands.waitSeconds(0.2))
                                                .andThen(Commands.runOnce(() -> controls.setOperatorRumble(0))));

                new Trigger(superstructure::isAligned)
                                .onTrue(Commands.runOnce(() -> controls.setDriverRumble(1))
                                                .andThen(Commands.waitSeconds(0.2))
                                                .andThen(Commands.runOnce(() -> controls.setDriverRumble(0))));
                

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}