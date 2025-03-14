// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Wrist;
import frc.robot.subsystems.claw.ClawConstants.Wrist.Positions;
import java.util.function.Supplier;

public class ClawPIDCommand extends Command {
  private final Claw mClawSubsystem;
  private double mSetpoint;
  private final ProfiledPIDController mPIDController;
  private ArmFeedforward mFeedforward;
  private boolean IS_TUNING = true;
  private double elevatorSetpoint;
  private double pivotPosition;
  private TrapezoidProfile.Constraints constraints;

  public ClawPIDCommand(Positions pSetpoint, Claw pClawSubsystem) {
    this(false, pSetpoint.getPos(), pClawSubsystem);
  }

  public ClawPIDCommand(Supplier<Positions> pSetpointSupplier, Claw pClawSubsystem) {
    this(false, pSetpointSupplier.get().getPos(), pClawSubsystem);
  }

  public ClawPIDCommand(boolean isTuning, double pSetpoint, Claw pClawSubsystem) {
    this.mClawSubsystem = pClawSubsystem;
    this.IS_TUNING = isTuning;

    this.constraints =
        new TrapezoidProfile.Constraints(
            ClawConstants.Wrist.kMaxVelocity, ClawConstants.Wrist.kMaxAcceleration);
    this.mPIDController = new ProfiledPIDController(Wrist.kP, 0.0, Wrist.kD, constraints);
    // new ProfiledPIDController(
    //     Wrist.kP,
    //     0.0,
    //     Wrist.kD,
    //     new TrapezoidProfile.Constraints(Wrist.kMaxVelocity, Wrist.kMaxAcceleration));
    this.mPIDController.setTolerance(Wrist.kTolerance);
    this.mFeedforward = new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);

    if (IS_TUNING) {
      this.mSetpoint = SmartDashboard.getNumber("TunableNumbers/Wrist/Setpoint", 0);
      System.out.println(
          String.format(
              "<<< %s - %s is in TUNING mode. >>>\n",
              this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
    } else {
      this.mSetpoint = MathUtil.clamp(pSetpoint, Wrist.kReverseSoftLimit, Wrist.kForwardSoftLimit);
    }

    this.elevatorSetpoint = SmartDashboard.getNumber("Elevator/Setpoint", 0);
    this.pivotPosition = SmartDashboard.getNumber("Pivot/Position", 0.0);
    SmartDashboard.putNumber("Tuning/Wrist/Output Value", 0.0);
    SmartDashboard.putNumber("Wrist/Setpoint", pSetpoint);

    addRequirements(pClawSubsystem);
  }

  @Override
  public void initialize() {
    double kP = SmartDashboard.getNumber("Wrist/Tuning/kP", ClawConstants.Wrist.kP);
    double kD = SmartDashboard.getNumber("Wrist/Tuning/kD", ClawConstants.Wrist.kD);
    double kG = SmartDashboard.getNumber("Wrist/Tuning/kG", ClawConstants.Wrist.kG);
    double kV = SmartDashboard.getNumber("Wrist/Tuning/kV", ClawConstants.Wrist.kV);
    double kA = SmartDashboard.getNumber("Wrist/Tuning/kA", ClawConstants.Wrist.kA);
    mPIDController.setPID(kP, 0, kD);
    // mPIDController.setConstraints(
    //     new TrapezoidProfile.Constraints(Wrist.kMaxVelocity, Wrist.kMaxAcceleration));
    // mPIDController.reset(getMeasurement());
    mPIDController.reset(
        new TrapezoidProfile.State(getMeasurement(), mClawSubsystem.getVelocity()));
    mFeedforward = new ArmFeedforward(ClawConstants.Wrist.kS, kG, kV, kA);
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    elevatorSetpoint = SmartDashboard.getNumber("Elevator/Setpoint", 0);
    if ((elevatorSetpoint <= 5) && (mSetpoint < 10)) {
      mSetpoint = 10;
      System.out.println(
          String.format(
              "<<< %s - %s is going too far down! >>>\n",
              this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
    }
    double calculatedFeedforward =
        mFeedforward.calculate(getMeasurement(), mPIDController.getSetpoint().velocity);
    double calculatedProfilePID = mPIDController.calculate(getMeasurement(), mSetpoint);
    double calculatedOutput = calculatedProfilePID + calculatedFeedforward;
    mClawSubsystem.setWrist(calculatedOutput);
    SmartDashboard.putNumber("Wrist/Setpoint", mSetpoint);
    SmartDashboard.putNumber("Wrist/Profiled Setpoint", mPIDController.getSetpoint().position);
    SmartDashboard.putNumber("Wrist/Profiled Velocity", mPIDController.getSetpoint().velocity);
    SmartDashboard.putNumber("Wrist/Output Value", calculatedOutput);
  }

  @Override
  public void end(boolean interrupted) {
    mClawSubsystem.setWrist(0);
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return mPIDController.atSetpoint();
  }

  private double getMeasurement() {
    return mClawSubsystem.getEncoderMeasurement();
  }
}
