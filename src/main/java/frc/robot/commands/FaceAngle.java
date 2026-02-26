package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FaceAngleConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;
public class FaceAngle extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Rotation2d m_targetAngle;
    private final ProfiledPIDController m_pidController;

    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric();

    public FaceAngle(CommandSwerveDrivetrain drivetrain, Rotation2d targetAngle) {
        m_drivetrain = drivetrain;
        m_targetAngle = targetAngle;

        m_pidController = new ProfiledPIDController(
                FaceAngleConstants.kP,
                FaceAngleConstants.kI,
                FaceAngleConstants.kD,
                new TrapezoidProfile.Constraints(
                        FaceAngleConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
                        FaceAngleConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ));

        m_pidController.enableContinuousInput(-Math.PI, Math.PI);
        m_pidController.setTolerance(FaceAngleConstants.ANGULAR_POSITION_TOLERANCE_RAD,
                                    FaceAngleConstants.ANGULAR_VELOCITY_TOLERANCE_RAD_PER_SEC);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_pidController.reset(m_drivetrain.getState().Pose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        double currentAngle = m_drivetrain.getState().Pose.getRotation().getRadians();
        double rotationalRate = m_pidController.calculate(currentAngle, m_targetAngle.getRadians());

        // Apply the rotational rate to the drivetrain. Linear velocities are zero.
        m_drivetrain.applyRequest(() -> m_drive.withRotationalRate(rotationalRate));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return m_pidController.atGoal();
    }
}
