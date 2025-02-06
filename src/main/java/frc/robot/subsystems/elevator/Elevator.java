// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax mElevatorSparkMax;
  private final SparkMaxConfig kElevatorConfig = new SparkMaxConfig();
  
  public Elevator() {
    mElevatorSparkMax = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);

    mElevatorSparkMax.configure(ElevatorConstants.kElevatorConfig, null, null);
    
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
