package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.HubTracker;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends SubsystemBase {
    private final Supplier<Pose2d> poseSupplier;
    public final Trigger activeHubTrigger = new Trigger(HubTracker::isActive).or(() -> HubTracker.getMatchTime() < 0);

    private boolean m_isAligned = false;

    public Superstructure(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public void setAligned(boolean isAligned) {
        m_isAligned = isAligned;
    }

    public boolean isAligned() {
        return m_isAligned;
    }

    public boolean inAllianceZone() {
        Pose2d pose = poseSupplier.get();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        return isBlue && pose.getMeasureX().lt(FieldConstants.ALLIANCE_ZONE)
                || !isBlue && pose.getMeasureX().gt(FieldConstants.FIELD_LENGTH.minus(FieldConstants.ALLIANCE_ZONE));
    }

    @Override
    public void periodic() {
        // Log if the robot is in the alliance zone to SmartDashboard
        SmartDashboard.putBoolean("Superstructure/InAllianceZone", inAllianceZone());
    }
}
