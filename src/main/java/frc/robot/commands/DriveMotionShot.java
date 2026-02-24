package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.AllianceUtil;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class DriveMotionShot extends Command {

        private final CommandSwerveDrivetrain drivetrain;
        private final ShooterSubsystem shooters;
        private final CommandXboxController driverController;

        private final ProfiledPIDController rotationController = new ProfiledPIDController(
                        Constants.DriveConstants.ALIGN_PID_P,
                        Constants.DriveConstants.ALIGN_PID_I,
                        Constants.DriveConstants.ALIGN_PID_D,
                        new TrapezoidProfile.Constraints(
                                        Constants.DriveConstants.ALIGN_MAX_VELOCITY_DEG_PER_SEC,
                                        Constants.DriveConstants.ALIGN_MAX_ACCEL_DEG_PER_SEC_SQ));

        private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.DriveConstants.DEADBAND * 5.0)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        public DriveMotionShot(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooters,
                        CommandXboxController driverController) {
                this.drivetrain = drivetrain;
                this.shooters = shooters;
                this.driverController = driverController;
                addRequirements(drivetrain, shooters);

                rotationController.enableContinuousInput(-180, 180);
                rotationController.setTolerance(Constants.DriveConstants.ALIGN_TOLERANCE_DEG,
                                Constants.DriveConstants.ALIGN_TOLERANCE_VEL_DEG_PER_SEC);
        }

        @Override
        public void initialize() {
                rotationController.reset(drivetrain.getState().Pose.getRotation().getDegrees());

                shooters.stopShooters();
        }

        @Override
        public void execute() {
                Pose2d robotPose = drivetrain.getState().Pose;
                ChassisSpeeds robotVel = drivetrain.getState().Speeds;

                Pose2d hubPose = AllianceUtil.getHubPose();

                Translation2d targetVec = hubPose.getTranslation().minus(robotPose.getTranslation());
                double distance = targetVec.getNorm();

                double timeOfFlight = Constants.ShooterConstants.distanceToTimeOfFlight.get(distance);

                Translation2d robotVelocityVec = new Translation2d(robotVel.vxMetersPerSecond,
                                robotVel.vyMetersPerSecond);
                Translation2d virtualTarget = hubPose.getTranslation().minus(robotVelocityVec.times(timeOfFlight));

                double targetYaw = Math.toDegrees(Math.atan2(
                                virtualTarget.getY() - robotPose.getY(),
                                virtualTarget.getX() - robotPose.getX()));

                double currentYaw = robotPose.getRotation().getDegrees();
                double rotOutput = rotationController.calculate(currentYaw, targetYaw);

                double setpointVel = rotationController.getSetpoint().velocity;
                double ff = setpointVel / Constants.DriveConstants.ALIGN_MAX_VELOCITY_DEG_PER_SEC * 0.1;

                ff += Math.signum(rotOutput) * Constants.DriveConstants.ALIGN_KS;

                double x = -driverController.getLeftY();
                double y = -driverController.getLeftX();

                x = Math.abs(x) > Constants.DriveConstants.DEADBAND ? x : 0.0;
                y = Math.abs(y) > Constants.DriveConstants.DEADBAND ? y : 0.0;

                double maxSpeed = 4.5;
                x *= maxSpeed;
                y *= maxSpeed;

                drivetrain.setControl(driveRequest
                                .withVelocityX(x)
                                .withVelocityY(y)
                                .withRotationalRate(Units.degreesToRadians(rotOutput + ff)));

                double effectiveDistance = virtualTarget.getDistance(robotPose.getTranslation());

                shooters.setTargetFromDistance(effectiveDistance);

                SmartDashboard.putNumber("AutoAlign/VirtualDist", effectiveDistance);
                SmartDashboard.putNumber("AutoAlign/TargetYaw", targetYaw);
        }

        @Override
        public void end(boolean interrupted) {
                shooters.stopShooters();
        }
}
