package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Notifier;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class AutoAimSysId {
  private final CommandSwerveDrivetrain drivetrain;
  private final PIDController pid;
  private final SlewRateLimiter slew;
  private final SwerveRequest.FieldCentric driveRequest;

  private Notifier notifier;
  private boolean running = false;
  private long startTimeMs;
  private FileWriter csvWriter;

  private final double[] testOffsets = {5.0, -5.0, 10.0, -10.0, 15.0, -15.0};
  private final double durationPerStep = 5.0;

  private int currentStep = 0;

  public AutoAimSysId(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.pid = new PIDController(Constants.KP, Constants.KI, Constants.KD);
    this.pid.enableContinuousInput(-180.0, 180.0);
    this.pid.setTolerance(Constants.PID_TOLERANCE_DEG);
    this.slew = new SlewRateLimiter(Constants.SLEW_RATE);
    this.driveRequest = new SwerveRequest.FieldCentric().withDriveRequestType(
        DriveRequestType.OpenLoopVoltage);
  }

  public void start() {
    if (running)
      return;
    running = true;

    try {
      String timestamp = LocalDateTime.now().format(
          DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
      csvWriter =
          new FileWriter("/home/lvuser/AutoAimSysId_" + timestamp + ".csv");
      csvWriter.write("Time,Step,Tx,Omega,Heading\n");
    } catch (IOException e) {
      e.printStackTrace();
    }

    currentStep = 0;
    startTimeMs = System.currentTimeMillis();

    notifier = new Notifier(this::runStep);
    notifier.startPeriodic(0.02);
  }

  private void runStep() {
    if (!running || currentStep >= testOffsets.length) {
      stop();
      return;
    }

    double stepStartTime =
        startTimeMs + (long)(currentStep * durationPerStep * 1000);
    double now = System.currentTimeMillis();
    double elapsed = (now - stepStartTime) / 1000.0;

    if (elapsed > durationPerStep) {

      currentStep++;
      if (currentStep >= testOffsets.length) {
        stop();
        return;
      }
      stepStartTime = now;
      elapsed = 0;
    }

    double tx = testOffsets[currentStep];

    double omega = pid.calculate(tx, 0.0);
    omega = MathUtil.clamp(omega, -Constants.MAX_OMEGA, Constants.MAX_OMEGA);
    omega = slew.calculate(omega);

    drivetrain.setControl(
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(
            omega));

    double headingDeg = drivetrain.getState().Pose.getRotation().getDegrees();

    try {
      double timeSinceStart = (now - startTimeMs) / 1000.0;
      csvWriter.write(String.format("%.3f,%d,%.3f,%.3f,%.3f\n", timeSinceStart,
                                    currentStep + 1, tx, omega, headingDeg));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void stop() {
    if (!running)
      return;
    running = false;

    drivetrain.setControl(
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

    try {
      csvWriter.flush();
      csvWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (notifier != null) {
      notifier.stop();
    }
    SignalLogger.writeString("AutoAimSysId", "Finished");
  }
}
