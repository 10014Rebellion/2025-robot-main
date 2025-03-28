// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.sensors.Potentiometer;

public class TelemetrySubsystem extends SubsystemBase {
  Field2d mField = new Field2d();

  public TelemetrySubsystem() {
    SmartDashboard.putData("RobotPose", mField);
  }

  public void add(String label, double value) {
    SmartDashboard.putNumber(label, value);
  }

  public void add(Pose2d pose) {
    mField.setRobotPose(pose);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Potentiometer Reading", Potentiometer.getPotentiometer());
  }
}
