// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Controllers extends SubsystemBase {
  /** Creates a new Controllers. */
  public Controllers() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  private ClawConstants.Wrist.Positions manipulatorToWrist(int level) {
    level = MathUtil.clamp(level, 0, 4);

    if (level == 4) {
      return ClawConstants.Wrist.Positions.L4;
    }

    if (level == 3) {
      return ClawConstants.Wrist.Positions.L3;
    }

    if (level == 2) {
      return ClawConstants.Wrist.Positions.L2;
    }

    return ClawConstants.Wrist.Positions.L1;
  }

  private ElevatorConstants.Positions manipulatorToElevator(int level) {
    level = MathUtil.clamp(level, 0, 4);
    if (level == 4) {
      return ElevatorConstants.Positions.L4;
    }

    if (level == 3) {
      return ElevatorConstants.Positions.L3;
    }

    if (level == 2) {
      return ElevatorConstants.Positions.L2;
    }
    return ElevatorConstants.Positions.L1;
  }
}
