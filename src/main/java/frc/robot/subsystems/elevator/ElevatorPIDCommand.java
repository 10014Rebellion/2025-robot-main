// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants.Positions;

public class ElevatorPIDCommand extends Command {
  private final Elevator mElevatorSubsystem;
  private double mSetpoint;
  private double clawSetpoint;
  private final ProfiledPIDController mProfiledPIDController;
  private final ElevatorFeedforward mElevatorFeedforward;

  private boolean IS_TUNING = true;

  public ElevatorPIDCommand(Positions pSetpoint, Elevator pElevatorSubsystem) {
    this(false, pSetpoint.getPos(), pElevatorSubsystem);
  }

  public ElevatorPIDCommand(boolean isTuning, double pSetpoint, Elevator pElevatorSubsystem) {
    this.IS_TUNING = isTuning;
    this.mElevatorSubsystem = pElevatorSubsystem;
    this.clawSetpoint = SmartDashboard.getNumber("TunableNumbers/Wrist/Tunable Setpoint", 0);
    this.mProfiledPIDController =
        new ProfiledPIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
    this.mElevatorFeedforward =
        new ElevatorFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
    this.mProfiledPIDController.setTolerance(ElevatorConstants.kTolerance);

    if (IS_TUNING) {
      this.mSetpoint = SmartDashboard.getNumber("TunableNumbers/Elevator/Tunable Setpoint", 0);
      System.out.println(
          String.format(
              "<<< %s - %s is in TUNING mode. >>>\n",
              this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName()));
    } else {
      this.mSetpoint =
          MathUtil.clamp(
              pSetpoint, ElevatorConstants.kReverseSoftLimit, ElevatorConstants.kForwardSoftLimit);
    }

    SmartDashboard.putNumber("Elevator/PID Output", 0.0);
    addRequirements(pElevatorSubsystem);
  }

  @Override
  public void initialize() {
    mProfiledPIDController.reset(getMeasurement());
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    // double potentiometerReading = Potentiometer.getPotentiometer();
    // mProfiledPIDController.setP(potentiometerReading);

    if (IS_TUNING) {
      mSetpoint = SmartDashboard.getNumber("TunableNumbers/Elevator/Tunable Setpoint", 0);
    }
    // This is an attempt to make it impossible for the elevator to go below some point and break
    // itself.
    // The values need to be tweaked
    // THIS IS TEMPORARY ISTG IF THIS IS STILL HERE IN A WEEK (currently 2/15/2025)
    if (clawSetpoint < 10 && mSetpoint < 5) {
      mSetpoint = 10;
      System.out.println(
          String.format(
              "<<< %s - %s is going too far down! >>>\n",
              this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName()));
    }

    double calculatedFeedforward = mElevatorFeedforward.calculate(0);
    double calculatedProfilePID = mProfiledPIDController.calculate(getMeasurement(), mSetpoint);
    double calculatedOutput = calculatedFeedforward + calculatedProfilePID;
    mElevatorSubsystem.setMotorVoltage(calculatedOutput);

    SmartDashboard.putNumber("Elevator/Calculated Output", calculatedOutput);
    SmartDashboard.putNumber("Elevator/Setpoint", mSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    mElevatorSubsystem.setMotorVoltage(0);
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    // return mProfiledPIDController.atGoal();
    return false;
  }

  private double getMeasurement() {
    return mElevatorSubsystem.getEncoderMeasurement();
  }
}
