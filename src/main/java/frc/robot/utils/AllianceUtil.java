package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Pose2d flipFieldAngle(Pose2d pose) {
    if (isRedAlliance()) {
      return new Pose2d(pose.getTranslation(), pose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    } else {
      return pose;
    }
  }

  public static Pose2d getHubPose() {
    return isRedAlliance() ? FieldPositions.getRedHubPose() : FieldPositions.getBlueHubPose();
  }

  public static double getDistanceToHub(frc.robot.subsystems.CommandSwerveDrivetrain drivetrain, String limelightName) {
    var estimate = frc.robot.utils.LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    if (estimate.tagCount > 0) {
      return estimate.pose.getTranslation().getDistance(getHubPose().getTranslation());
    }
    Pose2d robotPose = drivetrain.getState().Pose;
    return robotPose.getTranslation().getDistance(getHubPose().getTranslation());
  }

  public static double getTargetHeadingToHub(frc.robot.subsystems.CommandSwerveDrivetrain drivetrain,
      String limelightName) {
    var llResults = frc.robot.utils.LimelightHelpers.getLatestResults(limelightName);
    double currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();
    if (llResults.valid && llResults.targets_Fiducials.length > 0) {
      return currentHeading + llResults.tx;
    }
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d hubPose = getHubPose();
    double dx = hubPose.getX() - robotPose.getX();
    double dy = hubPose.getY() - robotPose.getY();
    Rotation2d angleToHub = new Rotation2d(Math.atan2(dy, dx));
    return angleToHub.getDegrees();
  }

}
