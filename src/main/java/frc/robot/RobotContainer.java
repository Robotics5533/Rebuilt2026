package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WasherSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.utils.Controls;
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
public class RobotContainer {
        // Constants for calculating max speed and angular rate based on configured
        // multipliers.
        private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                        * (Constants.DriveConstants.SPEED_MULTIPLIER / 100.0);
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond)
                        * (Constants.DriveConstants.SPEED_MULTIPLIER / 100.0);

        // SwerveRequest for field-centric driving with deadbands.
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * Constants.DriveConstants.DEADBAND)
                        .withRotationalDeadband(MaxAngularRate * Constants.DriveConstants.DEADBAND)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Telemetry for logging and SmartDashboard updates.
        private final Telemetry logger = new Telemetry(MaxSpeed);
        // Controls utility for handling driver and operator input.
        private final Controls controls = new Controls();
        // Field2d for visualizing robot position on the SmartDashboard.
        private final Field2d fieldViz = new Field2d();

        // Subsystems
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // Drivetrain subsystem
        private final LimelightSubsystem limelight = new LimelightSubsystem(Constants.LimelightConstants.LIMELIGHT_NAME,
                        drivetrain); // Limelight subsystem for vision processing
        // private final ClimbSubsystem climb = new ClimbSubsystem(); // Climb subsystem
        // (currently commented out)
        private final IntakeSubsystem intake = new IntakeSubsystem(); // Intake subsystem
        private final ShooterSubsystem shooters = new ShooterSubsystem(drivetrain,
                        Constants.LimelightConstants.LIMELIGHT_NAME); // Shooter subsystem
        private final WasherSubsystem washers = new WasherSubsystem(); // Washer subsystem
        private final FeederSubsystem feeder = new FeederSubsystem(); // Feeder subsystem
        private final Superstructure superstructure = new Superstructure(() -> drivetrain.getState().Pose);
        private final ClimbSubsystem climb = Constants.ClimbConstants.climbEnabled ? new ClimbSubsystem() : null; // Conditionally
                                                                                                                  // instantiate
                                                                                                                  // Climb
                                                                                                                  // subsystem

        // Autonomous command chooser for selecting auto routines on SmartDashboard.
        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure Limelight camera pose in robot space.
                // Todo Tune this, it's from the center of the robot in order its, forward
                // offset in meters, side offset in meters, up offset in meters, the rest are 0
                LimelightHelpers.setCameraPose_RobotSpace(
                                Constants.LimelightConstants.LIMELIGHT_NAME,
                                0.381,
                                0.3175,
                                0.079375,
                                0.0,
                                7.5,
                                0.0);

                // Register named commands for use in PathPlanner autonomous routines.
                NamedCommands.registerCommand("shoot_load",
                                new frc.robot.commands.ShootLoad(shooters, washers, feeder));
                NamedCommands.registerCommand("intake",
                                intake.runOnce(() -> intake.setFlip(IntakeSubsystem.FlipState.Out)));
                NamedCommands.registerCommand("start_intake", intake.run(intake::runRollerForward));
                NamedCommands.registerCommand("stop_intake", intake.runOnce(intake::stopRoller));

                // Build auto chooser and display on SmartDashboard.
                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);
                SmartDashboard.putData("Field", fieldViz);

                // Configure the button bindings
                configureBindings();
                // Warm up PathPlanner FollowPathCommand to reduce initial latency.
                FollowPathCommand.warmupCommand();
        }

        /**
         * Use this method to define your button->command mappings.
         */
        private void configureBindings() {

                // Set the default command for the drivetrain to field-centric control.
                drivetrain.setDefaultCommand(drivetrain.run(() -> {
                        drivetrain.setControl(
                                        drive.withVelocityX(controls.getDriveX() * MaxSpeed)
                                                        .withVelocityY(controls.getDriveY() * MaxSpeed)
                                                        .withRotationalRate(controls.getDriveOmega() * MaxAngularRate));

                }));

                // Configure driver and operator controls.
                controls.configureDriver(drivetrain, limelight, superstructure);
                if (climb != null) {
                        controls.configureOperator(drivetrain, intake, shooters, washers, feeder, superstructure,
                                        climb, limelight);
                } else {
                        // If climb is null, configure operator without climb
                        controls.configureOperator(drivetrain, intake, shooters, washers, feeder, superstructure, null, limelight);
                }

                // Default command for climb subsystem.
                if (climb != null) {
                        climb.setDefaultCommand(climb.run(climb::applySetpoint).ignoringDisable(true));
                }

                // Command to set drivetrain to idle when disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Configure neutral mode for drivetrain based on robot enable/disable state.
                RobotModeTriggers.disabled()
                                .onTrue(drivetrain.runOnce(() -> drivetrain.setNeutralMode(NeutralModeValue.Coast)));
                RobotModeTriggers.disabled().negate()
                                .onTrue(drivetrain.runOnce(() -> drivetrain.setNeutralMode(NeutralModeValue.Brake)));

                // Rumble the operator controller when shooters reach target speed.
                new Trigger(shooters::areShootersAtSpeed)
                                .onTrue(Commands.runOnce(() -> controls.setOperatorRumble(0.5))
                                                .andThen(Commands.waitSeconds(0.2))
                                                .andThen(Commands.runOnce(() -> controls.setOperatorRumble(0))));

                // Register drivetrain telemetry for logging.
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
