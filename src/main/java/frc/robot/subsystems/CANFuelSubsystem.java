// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANFuelSubsystem extends SubsystemBase {
  private static final int INTAKE_LAUNCHER_MOTOR_ID = 10;
  private static final int FEEDER_MOTOR_ID = 11;

  private static final double INTAKING_FEEDER_VOLTAGE = 6.0;
  private static final double INTAKING_INTAKE_VOLTAGE = 6.0;
  private static final double LAUNCHING_FEEDER_VOLTAGE = 8.0;
  private static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.0;
  private static final double SPIN_UP_FEEDER_VOLTAGE = -2.0;

  private static final int FEEDER_MOTOR_CURRENT_LIMIT = 40;
  private static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 40;

  private static final String KEY_INTAKE_FEEDER = "Intaking feeder roller value";
  private static final String KEY_INTAKE_LAUNCHER = "Intaking intake roller value";
  private static final String KEY_LAUNCH_FEEDER = "Launching feeder roller value";
  private static final String KEY_LAUNCH_LAUNCHER = "Launching launcher roller value";
  private static final String KEY_SPINUP_FEEDER = "Spin-up feeder roller value";

  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;

  public CANFuelSubsystem() {
    intakeLauncherRoller = createLauncherMotor();
    feederRoller = createFeederMotor();

    SmartDashboard.putNumber(KEY_INTAKE_FEEDER, INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(KEY_INTAKE_LAUNCHER, INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber(KEY_LAUNCH_FEEDER, LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(KEY_LAUNCH_LAUNCHER, LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber(KEY_SPINUP_FEEDER, SPIN_UP_FEEDER_VOLTAGE);
  }

  private SparkMax createFeederMotor() {
    SparkMax motor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    motor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
    return motor;
  }

  private SparkMax createLauncherMotor() {
    SparkMax motor = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    motor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
    return motor;
  }

  public void intake() {
    feederRoller.setVoltage(
        SmartDashboard.getNumber(KEY_INTAKE_FEEDER, INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(
        KEY_INTAKE_LAUNCHER, INTAKING_INTAKE_VOLTAGE));
  }

  public void eject() {
    feederRoller.setVoltage(-SmartDashboard.getNumber(
        KEY_INTAKE_FEEDER, INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(-SmartDashboard.getNumber(
        KEY_INTAKE_LAUNCHER, INTAKING_INTAKE_VOLTAGE));
  }

  public void launch() {
    feederRoller.setVoltage(
        SmartDashboard.getNumber(KEY_LAUNCH_FEEDER, LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(
        KEY_LAUNCH_LAUNCHER, LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public void spinUp() {
    feederRoller.setVoltage(SmartDashboard.getNumber(
        KEY_SPINUP_FEEDER, SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(
        KEY_LAUNCH_LAUNCHER, LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  public Command intakeCommand() {
    return runEnd(this::intake, this::stop);
  }

  public Command ejectCommand() {
    return runEnd(this::eject, this::stop);
  }

  public Command launchCommand() {
    return runEnd(this::launch, this::stop);
  }

  public Command spinUpCommand() {
    return runEnd(this::spinUp, this::stop);
  }
}
