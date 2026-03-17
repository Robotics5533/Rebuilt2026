package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;

@Logged
public class LimeLightSwitch {
    PowerDistribution PDH = new PowerDistribution(Constants.LimelightConstants.PDHCANID, ModuleType.kRev);

     public void setLimeLightModeOn(boolean on) {
        PDH.setSwitchableChannel(on);
    }
    public void setLimeLightModeOff(boolean off) {
        PDH.setSwitchableChannel(off);
    }
    
}
