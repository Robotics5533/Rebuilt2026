package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

import frc.robot.Constants;

import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {

  private final String name;
  private final NetworkTable telemetryTable;
  private final StructPublisher<Pose2d> posePublisher;

  private final CommandSwerveDrivetrain drivetrain;

  private final Notifier limelightNotifier_;

  private PoseEstimate lastPoseEstimate_ = new PoseEstimate();

  public LimelightSubsystem(String name, CommandSwerveDrivetrain drivetrain) {
    this.name = name;
    this.drivetrain = drivetrain;

    telemetryTable = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard/" + name);

    posePublisher = telemetryTable
        .getStructTopic("Estimated Robot Pose", Pose2d.struct)
        .publish();
    if (!SmartDashboard.containsKey("Vision/Enabled")) {
      SmartDashboard.putBoolean("Vision/Enabled", Constants.LimelightConstants.ENABLE_VISION_ODOMETRY);
    }

    limelightNotifier_ = new Notifier(this::updatePoseEstimate);
    limelightNotifier_.setName("limelight-poller");
    limelightNotifier_.startPeriodic(0.05); // 20 Hz
  }

  private void updatePoseEstimate() {
    lastPoseEstimate_ = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }   

  @Override
  public void periodic() {
    if (lastPoseEstimate_ != null && lastPoseEstimate_.tagCount > 0) {

      if (!SmartDashboard.getBoolean("Vision/Enabled", true)) {
        return;
      }

      double xyStdDev = 0.7;
      double degStdDev = 0.7;
      if (lastPoseEstimate_.tagCount >= 2) {
        xyStdDev = 0.1;
        degStdDev = 0.1;
      } else if (lastPoseEstimate_.avgTagDist < 4.0) {
        xyStdDev = 0.3;
        degStdDev = 0.3;
      }

      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, degStdDev));
      drivetrain.addVisionMeasurement(lastPoseEstimate_.pose, lastPoseEstimate_.timestampSeconds);
    }
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(name);
  }

  public double getTx() {
    return LimelightHelpers.getTX(name);
  }

  public double getTy() {
    return LimelightHelpers.getTY(name);
  }

  public double getTa() {
    return LimelightHelpers.getTA(name);
  }

}