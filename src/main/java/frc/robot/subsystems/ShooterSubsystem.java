package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leftMaster = new TalonFX(Constants.ShooterConstants.leftShooterMasterId);
  private final TalonFX leftFollower = new TalonFX(Constants.ShooterConstants.leftShooterFollowerId);
  private final TalonFX rightMaster = new TalonFX(Constants.ShooterConstants.rightShooterMasterId);
  private final TalonFX rightFollower = new TalonFX(Constants.ShooterConstants.rightShooterFollowerId);
  private final TalonFX leftWasher = new TalonFX(Constants.ShooterConstants.leftWasherMotorId);
  private final TalonFX rightWasher = new TalonFX(Constants.ShooterConstants.rightWasherMotorId);

  private final VelocityVoltage velocityCtrl = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageCtrl = new VoltageOut(0);
  private double lastLeftSetpointRps = 0.0;
  private double lastRightSetpointRps = 0.0;
  private final SysIdRoutine sysIdLeft = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (volts) -> leftMaster.setControl(new VoltageOut(volts)),
          (log) -> {
            log.motor("shooterLeft")
                .voltage(Units.Volts.of(leftMaster.getMotorVoltage().getValueAsDouble()))
                .angularVelocity(Units.RotationsPerSecond.of(leftMaster.getVelocity().getValueAsDouble()));
          },
          this));
  private final SysIdRoutine sysIdRight = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (volts) -> rightMaster.setControl(new VoltageOut(volts)),
          (log) -> {
            log.motor("shooterRight")
                .voltage(Units.Volts.of(rightMaster.getMotorVoltage().getValueAsDouble()))
                .angularVelocity(Units.RotationsPerSecond.of(rightMaster.getVelocity().getValueAsDouble()));
          },
          this));

  private final Mechanism2d mech = new Mechanism2d(2.0, 1.0);
  private final MechanismLigament2d leftLig = mech.getRoot("LeftRoot", 0.5, 0.5)
      .append(new MechanismLigament2d("Left", 0.4, 0, 4, new Color8Bit(Color.kBlue)));
  private final MechanismLigament2d rightLig = mech.getRoot("RightRoot", 1.5, 0.5)
      .append(new MechanismLigament2d("Right", 0.4, 0, 4, new Color8Bit(Color.kRed)));

  public ShooterSubsystem() {
    var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration()
        .withSlot0(new com.ctre.phoenix6.configs.Slot0Configs()
            .withKP(Constants.ShooterConstants.kP)
            .withKI(Constants.ShooterConstants.kI)
            .withKD(Constants.ShooterConstants.kD)
            .withKV(Constants.ShooterConstants.kV));
    leftMaster.getConfigurator().apply(cfg);
    leftFollower.getConfigurator().apply(cfg);
    rightMaster.getConfigurator().apply(cfg);
    rightFollower.getConfigurator().apply(cfg);
    leftMaster.setNeutralMode(NeutralModeValue.Coast);
    leftFollower.setNeutralMode(NeutralModeValue.Coast);
    rightMaster.setNeutralMode(NeutralModeValue.Coast);
    rightFollower.setNeutralMode(NeutralModeValue.Coast);
    leftWasher.setNeutralMode(NeutralModeValue.Brake);
    rightWasher.setNeutralMode(NeutralModeValue.Brake);
    SmartDashboard.putData("ShooterViz", mech);
  }

  public Command runLeftShooter() {
    return run(() -> {
      setLeftVelocity(Constants.ShooterConstants.shooterVelocityRPS);
      leftWasher.setControl(voltageCtrl.withOutput(Constants.ShooterConstants.washerVoltage));
      rightWasher.setControl(voltageCtrl.withOutput(-Constants.ShooterConstants.washerVoltage));
    }).finallyDo(() -> {
      setLeftVelocity(0);
      leftWasher.setControl(voltageCtrl.withOutput(0));
      rightWasher.setControl(voltageCtrl.withOutput(0));
    });
  }

  public Command runRightShooter() {
    return run(() -> {
      setRightVelocity(Constants.ShooterConstants.shooterVelocityRPS);
      leftWasher.setControl(voltageCtrl.withOutput(Constants.ShooterConstants.washerVoltage));
      rightWasher.setControl(voltageCtrl.withOutput(-Constants.ShooterConstants.washerVoltage));
    }).finallyDo(() -> {
      setRightVelocity(0);
      leftWasher.setControl(voltageCtrl.withOutput(0));
      rightWasher.setControl(voltageCtrl.withOutput(0));
    });
  }

  public Command runBothShooters() {
    return run(() -> {
      setLeftVelocity(Constants.ShooterConstants.shooterVelocityRPS);
      setRightVelocity(Constants.ShooterConstants.shooterVelocityRPS);
      leftWasher.setControl(voltageCtrl.withOutput(Constants.ShooterConstants.washerVoltage));
      rightWasher.setControl(voltageCtrl.withOutput(-Constants.ShooterConstants.washerVoltage));
    }).finallyDo(() -> {
      setLeftVelocity(0);
      setRightVelocity(0);
      leftWasher.setControl(voltageCtrl.withOutput(0));
      rightWasher.setControl(voltageCtrl.withOutput(0));
    });
  }

  public Command shootLoad(double reserveSeconds) {
    return run(() -> {
      setLeftVelocity(Constants.ShooterConstants.shooterVelocityRPS);
      setRightVelocity(Constants.ShooterConstants.shooterVelocityRPS);
      leftWasher.setControl(voltageCtrl.withOutput(Constants.ShooterConstants.washerVoltage));
      rightWasher.setControl(voltageCtrl.withOutput(-Constants.ShooterConstants.washerVoltage));
    }).until(() -> edu.wpi.first.wpilibj.DriverStation.getMatchTime() <= reserveSeconds)
        .finallyDo(() -> {
          setLeftVelocity(0);
          setRightVelocity(0);
          leftWasher.setControl(voltageCtrl.withOutput(0));
          rightWasher.setControl(voltageCtrl.withOutput(0));
        });
  }

  public void startBothShootersWasher() {
    setLeftVelocity(Constants.ShooterConstants.shooterVelocityRPS);
    setRightVelocity(Constants.ShooterConstants.shooterVelocityRPS);
    leftWasher.setControl(voltageCtrl.withOutput(Constants.ShooterConstants.washerVoltage));
    rightWasher.setControl(voltageCtrl.withOutput(-Constants.ShooterConstants.washerVoltage));
  }

  public void stopShootersWasher() {
    setLeftVelocity(0);
    setRightVelocity(0);
    leftWasher.setControl(voltageCtrl.withOutput(0));
    rightWasher.setControl(voltageCtrl.withOutput(0));
  }

  public Command runInterpolatedShot(DoubleSupplier distanceMeters) {
    return run(() -> {
      double d = distanceMeters.getAsDouble();
      double rps = Constants.ShooterConstants.distanceToVelocityRPS.get(d);
      setLeftVelocity(rps);
      setRightVelocity(rps);
      leftWasher.setControl(voltageCtrl.withOutput(Constants.ShooterConstants.washerVoltage));
      rightWasher.setControl(voltageCtrl.withOutput(-Constants.ShooterConstants.washerVoltage));
    }).finallyDo(() -> {
      setLeftVelocity(0);
      setRightVelocity(0);
      leftWasher.setControl(voltageCtrl.withOutput(0));
      rightWasher.setControl(voltageCtrl.withOutput(0));
    });
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdLeftQuasistaticForward() {
    return sysIdLeft.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdLeftQuasistaticReverse() {
    return sysIdLeft.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdLeftDynamicForward() {
    return sysIdLeft.dynamic(SysIdRoutine.Direction.kForward);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdLeftDynamicReverse() {
    return sysIdLeft.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdRightQuasistaticForward() {
    return sysIdRight.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdRightQuasistaticReverse() {
    return sysIdRight.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdRightDynamicForward() {
    return sysIdRight.dynamic(SysIdRoutine.Direction.kForward);
  }

  public edu.wpi.first.wpilibj2.command.Command sysIdRightDynamicReverse() {
    return sysIdRight.dynamic(SysIdRoutine.Direction.kReverse);
  }

  private void setLeftVelocity(double rps) {
    lastLeftSetpointRps = rps;
    leftMaster.setControl(velocityCtrl.withVelocity(rps));
    leftFollower.setControl(velocityCtrl.withVelocity(rps));
  }

  private void setRightVelocity(double rps) {
    lastRightSetpointRps = rps;
    rightMaster.setControl(velocityCtrl.withVelocity(rps));
    rightFollower.setControl(velocityCtrl.withVelocity(rps));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/LeftRPS", leftMaster.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/RightRPS", rightMaster.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/LeftSetpointRPS", lastLeftSetpointRps);
    SmartDashboard.putNumber("Shooter/RightSetpointRPS", lastRightSetpointRps);
    double leftLen = Math.min(0.8, Math.abs(leftMaster.getVelocity().getValueAsDouble()) / 120.0);
    double rightLen = Math.min(0.8, Math.abs(rightMaster.getVelocity().getValueAsDouble()) / 120.0);
    leftLig.setLength(0.2 + leftLen);
    rightLig.setLength(0.2 + rightLen);
  }
}
