package frc.robot.subsystems.claw;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Wrist;

public class NewClawPIDCommand extends Command {
  private final tempClaw mClawSubsystem;
  private double mSetpoint;
  private final ProfiledPIDController mPIDController;
  private ArmFeedforward mFeedforward;
  private TrapezoidProfile.Constraints constraints;

  public NewClawPIDCommand(double pSetpoint, tempClaw pClawSubsystem) {
    this.mClawSubsystem = pClawSubsystem;

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

    this.mSetpoint = pSetpoint;

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
    double calculatedFeedforward = mFeedforward.calculate(getMeasurement(), 0); // velocity setpoint

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
