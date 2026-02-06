package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class RobotConstants {
        public static final double robotOverallLength = Units.inchesToMeters(36.0);
        public static final double robotOverallWidth = Units.inchesToMeters(34.0);
    }

    public static final class DriveConstants {
        public static final double DEADBAND = 0.05;
        public static final double ALIGN_PID_P = 0.025;
        public static final double ALIGN_PID_I = 0.0;
        public static final double ALIGN_PID_D = 0.008;
        public static final double ALIGN_TOLERANCE_DEG = 2.0;
        public static final double ALIGN_ROTATION_LIMIT = 4.0;


        // PathPlanner PID Constants
        public static final double AUTO_DRIVE_P = 15.0;
        public static final double AUTO_DRIVE_I = 0.0;
        public static final double AUTO_DRIVE_D = 1.1;

        public static final double AUTO_ROTATION_P = 12.0;
        public static final double AUTO_ROTATION_I = 0.0;
        public static final double AUTO_ROTATION_D = 1.0;
    }

    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double TRIGGER_THRESHOLD = 0.5;
    }

    public static final class LimelightConstants {
        public static final Set < Integer > RED_HUB_TAGS = Set.of(11, 2, 9, 10, 8, 5);
        public static final Set < Integer > BLUE_HUB_TAGS = Set.of(18, 27, 26, 25, 21, 24);
        public static final String LIMELIGHT_NAME = "limelight";
    }

    public static final class ClimbConstants {
        public static final int climbMotorId = 20;
        // inches per motor rotation:
        //  - Drum: (PI * drumDiameterInches) / motorToDrumGearRatio
        //  - Leadscrew: (leadPitchInchesPerRev) / motorToScrewGearRatio
        public static final double rotationsToInches = 1.0;
        public static final double inactivePositionInches = 0.0;
        public static final double activePositionInches = 6.5;
        public static final double hangingPositionInches = 5.25;
        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double maxVoltage = 10.0;
    }

    public static final class IntakeConstants {
        public static final int intakeFlipMotorId = 21;
        public static final int intakeRollerMotorId = 22;
        public static final double flipInPositionRot = 0.0;
        public static final double flipOutPositionRot = 25.0;
        public static final double flipToleranceRot = 0.5;
        public static final double flipMaxVelocityRotPerS = 30.0;
        public static final double flipMaxAccelRotPerSSq = 60.0;
        public static final double flipkP = 0.1475;
        public static final double flipkI = 0.0;
        public static final double flipkD = 0.0;
        public static final double flipkV = 2.7668;
        public static final double rollerVoltage = 8.0;
    }

    public static final class ShooterConstants {
        public static final int leftShooterMasterId = 23;
        public static final int leftShooterFollowerId = 24;
        public static final int rightShooterMasterId = 25;
        public static final int rightShooterFollowerId = 26;
        public static final int leftWasherMotorId = 27;
        public static final int rightWasherMotorId = 28;
        public static final double shooterVelocityRPS = 80.0;
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.12;
        public static final double washerVoltage = 6.0;
        public static final InterpolatingDoubleTreeMap distanceToVelocityRPS = new InterpolatingDoubleTreeMap();
        static {
            distanceToVelocityRPS.put(2.5, 70.0);
            distanceToVelocityRPS.put(3.0, 75.0);
            distanceToVelocityRPS.put(3.5, 80.0);
            distanceToVelocityRPS.put(4.0, 85.0);
            distanceToVelocityRPS.put(4.5, 90.0);
        }
    }

    public static final class FieldConstants {

        static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
        public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

        static final double blueStartingLineX = Units.inchesToMeters(156);
        static final double blueOutpostSideTrenchStartY = Units.inchesToMeters(24.85);
        static final double blueDepotSideTrenchStartY = fieldWidth - Units.inchesToMeters(24.85);
        static final double bumpWidth = Inches.of(73.0).in(Meters);
        static final double bumpDepth = Inches.of(44.4).in(Meters);

        public static final Pose2d blueOutpostSideTrenchStart = new Pose2d(
            blueStartingLineX - RobotConstants.robotOverallLength / 2.0,
            blueOutpostSideTrenchStartY,
            new Rotation2d());

        public static final Pose2d blueDepotSideTrenchStart = new Pose2d(
            blueStartingLineX - RobotConstants.robotOverallLength / 2.0,
            blueDepotSideTrenchStartY,
            new Rotation2d());

        public static final Pose2d blueCenterStart = new Pose2d(
            blueStartingLineX - RobotConstants.robotOverallLength / 2.0,
            fieldWidth / 2.0,
            new Rotation2d());

        static final double hubCenterFromAllianceWall = Units.inchesToMeters(158.6);
        static final double bumpCenterFromAllianceWall = Units.inchesToMeters(95.25);
        static final double hubToBumpCenterOffset = Units.inchesToMeters(90.0);

        public static Pose2d blueHubPose = new Pose2d(
            Units.inchesToMeters(181.56), FieldConstants.fieldWidth / 2, new Rotation2d());

        static double redStartingLineX = Units.inchesToMeters(fieldLength - Units.inchesToMeters(143.5));

        public static Pose2d redHubPose = new Pose2d(
            FieldConstants.fieldLength - Units.inchesToMeters(181.56), FieldConstants.fieldWidth / 2,
            new Rotation2d(Math.PI));

        public static Pose2d blueTowerRightPose = new Pose2d(
            blueHubPose.getX() - Units.inchesToMeters(151),
            blueHubPose.getY() - Units.inchesToMeters(45),
            new Rotation2d());

        public static Pose2d redTowerRightPose = new Pose2d(
            redHubPose.getX() + Units.inchesToMeters(151),
            redHubPose.getY() + Units.inchesToMeters(45),
            new Rotation2d(Math.PI));

        public static final Pose2d blueBumpLeftPose = new Pose2d(
            blueHubPose.getX(),
            blueHubPose.getY() + hubToBumpCenterOffset + Units.inchesToMeters(2.5),
            new Rotation2d());

        public static final Pose2d blueBumpRightPose = new Pose2d(
            blueHubPose.getX(),
            blueHubPose.getY() - hubToBumpCenterOffset,
            new Rotation2d());

        public static final Pose2d redBumpLeftPose = new Pose2d(
            redHubPose.getX(),
            redHubPose.getY() - hubToBumpCenterOffset,
            new Rotation2d(Math.PI));

        public static final Pose2d redBumpRightPose = new Pose2d(
            redHubPose.getX(),
            redHubPose.getY() + hubToBumpCenterOffset + Units.inchesToMeters(2.5),
            new Rotation2d(Math.PI));

    }
}
