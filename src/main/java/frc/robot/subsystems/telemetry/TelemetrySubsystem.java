// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.sensors.Potentiometer;

public class TelemetrySubsystem extends SubsystemBase {

  HttpCamera frontRight =
      new HttpCamera("PhotonVisionCamera", "http://10.100.14.99:1182/stream.mjpg");
  HttpCamera frontLeft =
      new HttpCamera("PhotonVisionCamera", "http://10.100.14.99:1184/stream.mjpg");

  public TelemetrySubsystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Potentiometer Reading", Potentiometer.getPotentiometer());
  }
}
