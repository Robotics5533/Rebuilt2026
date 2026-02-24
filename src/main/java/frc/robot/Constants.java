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
import edu.wpi.first.units.measure.Distance;

public final class Constants {

    public static final class RobotConstants {
        public static final double robotOverallLength = Units.inchesToMeters(36.0);
        public static final double robotOverallWidth = Units.inchesToMeters(34.0);
    }

    public static final class DriveConstants {
        public static final double DEADBAND = 0.05;
        public static final double SPEED_MULTIPLIER = 100.0; 

        public static final double ALIGN_PID_P = 0.045;
        public static final double ALIGN_PID_I = 0.05;
        public static final double ALIGN_PID_D = 0.002;

        public static final double ALIGN_KS = 0.04;
        public static final double ALIGN_TOLERANCE_DEG = 0.5;
        public static final double ALIGN_TOLERANCE_VEL_DEG_PER_SEC = 10.0;

        public static final double ALIGN_MAX_VELOCITY_DEG_PER_SEC = 250.0;
        public static final double ALIGN_MAX_ACCEL_DEG_PER_SEC_SQ = 300.0;

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
        public static final double TRIGGER_THRESHOLD = 0.3;
    }

    public static final class LimelightConstants {
        public static final Set<Integer> RED_HUB_TAGS = Set.of(11, 2, 9, 10, 8, 5);
        public static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 27, 26, 25, 21, 24);
        public static final String LIMELIGHT_NAME = "limelight";

        public static final boolean ENABLE_VISION_ODOMETRY = true;
        public static final boolean TEST_MODE = false;
        public static final double VISION_REJECTION_SPEED_THRESHOLD_MPS = 2.0;
        public static final double VISION_REJECTION_DISTANCE_THRESHOLD_METERS = 1.0;
    }

    public static final class ClimbConstants {
        public static final int climbMotorId = 60;

        public static final double climbGearRatio = 125.0;
        public static final double motorFreeSpeedRPM = 7500.0;

        public static final double sprocketCircumferenceInches = 4.5;

        public static final double motorFreeSpeedRPS = motorFreeSpeedRPM / 60.0;
        public static final double mechanismMaxRPS = motorFreeSpeedRPS / climbGearRatio;
        public static final double mechanismMaxLinearVelocity = mechanismMaxRPS * sprocketCircumferenceInches;
        public static final double rotationsToInches = mechanismMaxLinearVelocity / mechanismMaxRPS;

        public static final double climbCruiseVelocityRPS = 20.0;
        public static final double rampTimeSeconds = 0.05;
        public static final double climbAccelerationRPS2 = climbCruiseVelocityRPS / rampTimeSeconds;

        public static final double inactivePositionInches = 0.0;
        public static final double activePositionInches = 6.5;
        public static final double hangingPositionInches = 5.25;
        public static final double kP = 80.0;
        public static final double kI = 0.5;
        public static final double kD = 1.0;
        public static final double kG = 0.0;
        public static final double maxVoltage = 12.0;
        public static final double climbCurrentLimit = 40.0;
        public static final double softLimitForwardInches = 7.0;
        public static final double softLimitReverseInches = 0.0;
        public static final boolean climbEnabled = true;
    }

    public static final class IntakeConstants {
        public static final int intakeFlipMotorId = 14;
        public static final int intakeRollerMotorId = 21;
        public static final double flipGearRatio = 120.0;
        public static final double flipInPositionDeg = -5.0;
        public static final double flipStowPositionDeg = 50.0;
        public static final double flipOutPositionDeg = 125.0;
        public static final double flipToleranceDeg = 2.0;
        public static final double flipMaxVelocityRotPerS = 3.0;
        public static final double flipMaxAccelRotPerSSq = 6.0;
        public static final double flipkP = 40.0;
        public static final double flipkI = 1.0;
        public static final double flipkD = 0.0;
        public static final double flipkV = 0.0;
        public static final double flipkA = 0.0;
        public static final double flipCurrentLimit = 40.0;
        public static final double rollerVoltage = 12.0;
        public static final double softLimitForwardDeg = 95.0;
        public static final double softLimitReverseDeg = -5.0;
    }

    public static final class ShooterConstants {
        public static final int leftShooterId = 17;
        public static final int leftFeederId = 18;
        public static final int rightShooterId = 20;
        public static final int rightFeederId = 19;
        public static final double feederVoltage = 8.0;
        public static final double feederCurrentLimit = 20.0;
        public static final int leftWasherMotorId = 16;
        public static final int rightWasherMotorId = 15;
        public static final double shooterVelocityRPS = 50;
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.12;
        public static final double washerVoltage = 6.0;
        public static final double shooterTargetVoltage = 12.0;
        public static final double shooterSpeedThresholdRPS = 200.0;
        public static final double shootAtDistanceDurationSeconds = 3.0;
        public static final double shooterCurrentLimit = 40.0;
        public static final double fuelApproxVelocityMps = 15.0;

        /**
         * Represents a single data point for shooter calibration, mapping distance to
         * target RPS, voltage, and time of flight.
         */
        public static class ShotData {
            public final double distanceMeters;
            public final double rps;
            public final double voltage;
            public final double timeOfFlightSeconds;

            public ShotData(double distanceMeters, double rps, double voltage, double timeOfFlightSeconds) {
                this.distanceMeters = distanceMeters;
                this.rps = rps;
                this.voltage = voltage;
                this.timeOfFlightSeconds = timeOfFlightSeconds;
            }
        }

        public static final ShotData SHOT_1 = new ShotData(1, 65.0, 7.0, 0);
        public static final ShotData SHOT_2 = new ShotData(1.5, 60.0, 7.5, 0.45);
        public static final ShotData SHOT_3 = new ShotData(2.0, 65.0, 7.8, 0.48);
        public static final ShotData SHOT_4 = new ShotData(2.5, 70.0, 8.0, 0.5);
        public static final ShotData SHOT_5 = new ShotData(3.0, 75.0, 8.5, 0.6);
        public static final ShotData SHOT_6 = new ShotData(3.5, 80.0, 9.0, 0.7);
        public static final ShotData SHOT_7 = new ShotData(4.0, 85.0, 9.5, 0.8);
        public static final ShotData SHOT_8 = new ShotData(4.5, 90.0, 10.0, 0.9);
        public static final ShotData SHOT_9 = new ShotData(5.0, 95.0, 10.5, 1.0);
        public static final ShotData SHOT_10 = new ShotData(5.5, 100.0, 11.0, 1.1);

        public static final InterpolatingDoubleTreeMap distanceToVelocityRPS = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToVoltage = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToTimeOfFlight = new InterpolatingDoubleTreeMap();

        static {
            addShotData(SHOT_1);
            addShotData(SHOT_2);
            addShotData(SHOT_3);
            addShotData(SHOT_4);
            addShotData(SHOT_5);
            addShotData(SHOT_6);
            addShotData(SHOT_7);
            addShotData(SHOT_8);
            addShotData(SHOT_9);
            addShotData(SHOT_10);
        }

        private static void addShotData(ShotData data) {
            distanceToVelocityRPS.put(data.distanceMeters, data.rps);
            distanceToVoltage.put(data.distanceMeters, data.voltage);
            distanceToTimeOfFlight.put(data.distanceMeters, data.timeOfFlightSeconds);
        }
    }

    public static final class FieldConstants {

        static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltAndymark);
        public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
        public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

        public static final Distance ALLIANCE_ZONE = Inches.of(156.06);
         public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

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
                FieldConstants.fieldLength - Units.inchesToMeters(181.56),
                FieldConstants.fieldWidth / 2,
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
