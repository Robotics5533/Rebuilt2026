// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    File tracyLib = new File(System.getProperty("user.dir"), "libs/x86_64/libtracy-jni-amd64.so");

    if (tracyLib.exists()) {
        try {
            // 2. Explicitly load the file. This bypasses the need for java.library.path
            System.load(tracyLib.getAbsolutePath());
            System.out.println("********** Tracy JNI loaded successfully **********");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("CRITICAL: Could not load Tracy JNI: " + e.getMessage());
        }
    } else {
        System.err.println("CRITICAL: Tracy library not found at: " + tracyLib.getAbsolutePath());
    }
    RobotBase.startRobot(Robot::new);
  }
}