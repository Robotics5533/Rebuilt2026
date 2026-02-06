package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PIDTuningLogger {
    private final NetworkTable table;
    private final StructSubscriber<Pose2d> currentPoseSub;
    private final StructSubscriber<Pose2d> targetPoseSub;

    private final List<Double> xyErrors = new ArrayList<>();
    private final List<Double> thetaErrors = new ArrayList<>();

    private boolean isRecording = false;

    public PIDTuningLogger() {
        table = NetworkTableInstance.getDefault().getTable("PathPlanner");

        currentPoseSub = table.getStructTopic("currentPose", Pose2d.struct).subscribe(null);
        targetPoseSub = table.getStructTopic("targetPose", Pose2d.struct).subscribe(null);
    }

    public void start() {
        xyErrors.clear();
        thetaErrors.clear();
        isRecording = true;

        System.out.println("[PIDTuningLogger] Started recording PathPlanner telemetry (Manual Calculation Mode)...");
    }

    public void update() {
        if (!isRecording)
            return;

        Pose2d current = currentPoseSub.get();
        Pose2d target = targetPoseSub.get();

        if (current != null && target != null) {

            double xy = current.getTranslation().getDistance(target.getTranslation());

            double theta = Math.abs(current.getRotation().minus(target.getRotation()).getRadians());

            xyErrors.add(xy);
            thetaErrors.add(theta);
        }
    }

    public void stopAndPrintReport() {
        if (!isRecording)
            return;
        isRecording = false;

        if (xyErrors.isEmpty()) {
            System.out
                    .println("[PIDTuningLogger] No data collected. (Check if Auto is running and Poses are published)");
            return;
        }

        double avgXy = calculateAverage(xyErrors);
        double maxXy = Collections.max(xyErrors);

        double avgTheta = calculateAverage(thetaErrors);
        double maxTheta = Collections.max(thetaErrors);

        System.out.println("\n==================================================");
        System.out.println("       PID TUNING DATA REPORT       ");
        System.out.println("==================================================");
        System.out.println("<PID_REPORT>");
        System.out.printf("XY_Error_Avg: %.4f%n", avgXy);
        System.out.printf("XY_Error_Max: %.4f%n", maxXy);
        System.out.printf("Theta_Error_Avg: %.4f%n", avgTheta);
        System.out.printf("Theta_Error_Max: %.4f%n", maxTheta);
        System.out.printf("Samples: %d%n", xyErrors.size());
        System.out.println("</PID_REPORT>");
        System.out.println("==================================================\n");
    }

    private double calculateAverage(List<Double> marks) {
        if (marks == null || marks.isEmpty()) {
            return 0;
        }
        double sum = 0;
        for (Double mark : marks) {
            sum += mark;
        }
        return sum / marks.size();
    }
}
