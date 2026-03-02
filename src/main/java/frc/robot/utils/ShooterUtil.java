package frc.robot.utils;

import java.lang.reflect.Array;

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

    public static final ShooterShotData[] SHOT_DATA = {
        new ShooterShotData(1.0, 60, 4.8, 0.20),
        new ShooterShotData(2.0, 56.0, 5.7, 0.28),
        new ShooterShotData(3.0, 70.0, 6.6, 0.35),
        new ShooterShotData(4.0, 80.0, 7.6, 0.43),
        new ShooterShotData(4.5, 85.0, 9.9, 0.61),
        new ShooterShotData(5.0, 90.0, 8.7, 0.52),
        new ShooterShotData(6.0, 105.0, 9.9, 0.61),
    };


    /**
     * Calcuates the ideal target velocity in m/s to a specified distance and height away, 1.2827m is delta H
     * @param distance
     * @param height
     * @param gravity
     * @param shotAngleRadians
     * @return
     */
    public static double calculateTargetVelocity(double distance, double height, double gravity, double shotAngleRadians){
        return Math.sqrt((gravity*distance*distance)/(2.0*Math.pow(Math.cos(shotAngleRadians),2)*(height-distance*Math.tan(shotAngleRadians))));
    }


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

        
        for (ShooterShotData shooterShotData : SHOT_DATA) {
            addShotData(distanceToVelocityRPS, distanceToVoltage, distanceToTimeOfFlight, shooterShotData);
        }
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

