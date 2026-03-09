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


    public static final ShooterShotData SHOT_1_METER = new ShooterShotData(1.0, 60, 4.8, 0.85);
    public static final ShooterShotData SHOT_2_METER = new ShooterShotData(2.0, 56.0, 5.7, 0.98);
    public static final ShooterShotData SHOT_3_METER = new ShooterShotData(3.0, 70.0, 6.6, 1.15);
    public static final ShooterShotData SHOT_4_METER = new ShooterShotData(4.0, 80.0, 7.6, 1.22);
    public static final ShooterShotData SHOT_4HALF_METER = new ShooterShotData(4.5, 85.0, 9.9, 1.23);
    public static final ShooterShotData SHOT_5_METER = new ShooterShotData(5.0, 90.0, 8.7, 1.25);
    public static final ShooterShotData SHOT_6_METER = new ShooterShotData(6.0, 105.0, 9.9, 1.30);


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
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_4HALF_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_5_METER);
        addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, SHOT_6_METER);
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

