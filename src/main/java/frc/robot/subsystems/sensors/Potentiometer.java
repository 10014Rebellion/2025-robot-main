// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Potentiometer {
  private static AnalogPotentiometer mPotentiometer = new AnalogPotentiometer(1);

  public static double getPotentiometer() {
    return mPotentiometer.get();
  }
}
