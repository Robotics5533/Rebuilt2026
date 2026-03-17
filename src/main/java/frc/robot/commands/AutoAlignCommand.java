package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;

@Logged
public class AutoAlignCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

  private final ProfiledPIDController alignPID = new ProfiledPIDController(
      Constants.DriveConstants.ALIGN_PID_P,
      Constants.DriveConstants.ALIGN_PID_I,
      Constants.DriveConstants.ALIGN_PID_D,
      new TrapezoidProfile.Constraints(
          Constants.DriveConstants.ALIGN_MAX_VELOCITY_DEG_PER_SEC,
          Constants.DriveConstants.ALIGN_MAX_ACCEL_DEG_PER_SEC_SQ));

  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private boolean finishAtSetpoint = false;

  private Supplier<Rotation2d> m_targetRotationSupplier;
  private Supplier<Double> m_vxSupplier;
  private Supplier<Double> m_vySupplier;
  private Consumer<Boolean> m_alignedConsumer;

  public AutoAlignCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Rotation2d> targetRotationSupplier) {
    this(drivetrain, targetRotationSupplier, () -> 0.0, () -> 0.0, (aligned) -> {});
  }

  public AutoAlignCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Rotation2d> targetRotationSupplier,
      Supplier<Double> vxSupplier,
      Supplier<Double> vySupplier) {
        this(drivetrain, targetRotationSupplier, vxSupplier, vySupplier, (aligned) -> {});
      }

  public AutoAlignCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Rotation2d> targetRotationSupplier,
      Supplier<Double> vxSupplier,
      Supplier<Double> vySupplier,
      Consumer<Boolean> alignedConsumer) {
    this.drivetrain = drivetrain;
    this.m_targetRotationSupplier = targetRotationSupplier;
    this.m_vxSupplier = vxSupplier;
    this.m_vySupplier = vySupplier;
    this.m_alignedConsumer = alignedConsumer;

    addRequirements(drivetrain);

    SmartDashboard.putNumber("AutoAlign/kP", Constants.DriveConstants.ALIGN_PID_P);
    SmartDashboard.putNumber("AutoAlign/kI", Constants.DriveConstants.ALIGN_PID_I);
    SmartDashboard.putNumber("AutoAlign/kD", Constants.DriveConstants.ALIGN_PID_D);
    SmartDashboard.putNumber("AutoAlign/kS", Constants.DriveConstants.ALIGN_KS);
  }

  public AutoAlignCommand finishWhenAligned() {
    this.finishAtSetpoint = true;
    return this;
  }

  @Override
  public void initialize() {

    double kP = SmartDashboard.getNumber("AutoAlign/kP", Constants.DriveConstants.ALIGN_PID_P);
    double kI = SmartDashboard.getNumber("AutoAlign/kI", Constants.DriveConstants.ALIGN_PID_I);
    double kD = SmartDashboard.getNumber("AutoAlign/kD", Constants.DriveConstants.ALIGN_PID_D);

    alignPID.setPID(kP, kI, kD);
    alignPID.enableContinuousInput(-180, 180);

    alignPID.setTolerance(
        Constants.DriveConstants.ALIGN_TOLERANCE_DEG,
        Constants.DriveConstants.ALIGN_TOLERANCE_VEL_DEG_PER_SEC);
    alignPID.reset(drivetrain.getState().Pose.getRotation().getDegrees());
  }

  @Override
  public void execute() {

    double currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();
    double targetAngle = m_targetRotationSupplier.get().getDegrees();

    double pidOutput = alignPID.calculate(currentHeading, targetAngle);

    double setpointVelocity = alignPID.getSetpoint().velocity;
    double maxVelDeg = Constants.DriveConstants.ALIGN_MAX_VELOCITY_DEG_PER_SEC;
    double feedforward = setpointVelocity / maxVelDeg;

    double kS = SmartDashboard.getNumber("AutoAlign/kS", Constants.DriveConstants.ALIGN_KS);
    double totalOutput = pidOutput + feedforward;

    if (Math.abs(totalOutput) > 0.001) {
      totalOutput += Math.signum(totalOutput) * kS;
    }

    SmartDashboard.putNumber("AutoAlign/CurrentAngle", currentHeading);
    SmartDashboard.putNumber("AutoAlign/TargetAngle", targetAngle);
    SmartDashboard.putNumber("AutoAlign/Error", alignPID.getPositionError());
    SmartDashboard.putNumber("AutoAlign/Output", totalOutput);
    SmartDashboard.putBoolean("AutoAlign/AtSetpoint", alignPID.atSetpoint());
    SmartDashboard.putNumber("AutoAlign/ProfileVelocity", setpointVelocity);

    m_alignedConsumer.accept(alignPID.atSetpoint());

    drivetrain.setControl(driveRequest
        .withVelocityX(m_vxSupplier.get())
        .withVelocityY(m_vySupplier.get())
        .withRotationalRate(totalOutput * maxAngularRate));
  }

  @Override
  public void end(boolean interrupted) {
    m_alignedConsumer.accept(false);
    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public boolean isFinished() {
    return finishAtSetpoint && alignPID.atSetpoint();
  }

}