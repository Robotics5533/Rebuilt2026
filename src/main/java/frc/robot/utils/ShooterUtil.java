package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterUtil {

    public static final double SHOOTER_KV_RPS_PER_VOLT = 60.0 / 10.0;
    /**
     * Represents a set of shooter parameters for a specific shot distance.
     * Contains target RPS, target Voltage, and estimated Time of Flight.
     */
    public static class ShooterShotData {
        public final double distanceMeters;
        public final double targetRPS;
        public final double targetVoltage;
        public final double timeOfFlight;

        public ShooterShotData(double distanceMeters, double targetRPS, double targetVoltage, double timeOfFlight) {
            this.distanceMeters = distanceMeters;
            this.targetRPS = targetRPS;
            this.targetVoltage = targetVoltage;
            this.timeOfFlight = timeOfFlight;
        }
    }


    public static final ShooterShotData SHOT_1_METER = new ShooterShotData(1.0, 47.5, 4.8, 0.20);
    public static final ShooterShotData SHOT_2_METER = new ShooterShotData(2.0, 56.0, 5.7, 0.28);
    public static final ShooterShotData SHOT_3_METER = new ShooterShotData(3.0, 65.0, 6.6, 0.35);
    public static final ShooterShotData SHOT_4_METER = new ShooterShotData(4.0, 75.0, 7.6, 0.43);
    public static final ShooterShotData SHOT_5_METER = new ShooterShotData(5.0, 86.0, 8.7, 0.52);
    public static final ShooterShotData SHOT_6_METER = new ShooterShotData(6.0, 98.0, 9.9, 0.61);
    public static final ShooterShotData SHOT_7_METER = new ShooterShotData(7.0, 111.0, 11.2, 0.71);
    public static final ShooterShotData SHOT_8_METER = new ShooterShotData(8.0, 125.0, 12.6, 0.82);
    public static final ShooterShotData SHOT_9_METER = new ShooterShotData(9.0, 140.0, 14.1, 0.94);
    public static final ShooterShotData SHOT_10_METER = new ShooterShotData(10.0, 156.0, 15.7, 1.07);


    /**
     * Populates the interpolation maps with predefined shot data.
     * This method should be called once, for example, in Constants.java static block.
     *
     * @param distanceToVelocityRPS The map to populate with distance-to-RPS data.
     * @param distanceToVoltage The map to populate with distance-to-Voltage data.
     * @param distanceToTimeOfFlight The map to populate with distance-to-TimeOfFlight data.
     */
    public static void populateShotDataMaps(
            InterpolatingDoubleTreeMap distanceToVelocityRPS,
            InterpolatingDoubleTreeMap distanceToVoltage,
            InterpolatingDoubleTreeMap distanceToTimeOfFlight) {
        
       
        distanceToVelocityRPS.clear();
        distanceToVoltage.clear();
        distanceToTimeOfFlight.clear();

        
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_1_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_2_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_3_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_4_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_5_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_6_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_7_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_8_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_9_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_10_METER);
    }

    private static void addShotData(
            InterpolatingDoubleTreeMap distanceToVelocityRPS,
            InterpolatingDoubleTreeMap distanceToVoltage,
            InterpolatingDoubleTreeMap distanceToTimeOfFlight,
            ShooterShotData shot) {
        distanceToVelocityRPS.put(shot.distanceMeters, shot.targetRPS);
        distanceToVoltage.put(shot.distanceMeters, shot.targetVoltage);
        distanceToTimeOfFlight.put(shot.distanceMeters, shot.timeOfFlight);
    }
}

