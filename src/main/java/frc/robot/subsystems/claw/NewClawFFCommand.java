package frc.robot.subsystems.claw;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Wrist;

public class NewClawFFCommand extends Command {
  private final tempClaw mClawSubsystem;
  private ArmFeedforward mClawFeedforward;
  private double pivotPosition;

  public NewClawFFCommand(tempClaw pClawSubsystem) {
    this.mClawSubsystem = pClawSubsystem;
    this.mClawFeedforward = new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);
    this.pivotPosition = SmartDashboard.getNumber("Pivot/Position", 0.0);
    addRequirements(pClawSubsystem);
  }

  @Override
  public void initialize() {
    double kG = SmartDashboard.getNumber("Wrist/Tuning/kG", ClawConstants.Wrist.kG);
    double kV = SmartDashboard.getNumber("Wrist/Tuning/kV", ClawConstants.Wrist.kV);
    double kA = SmartDashboard.getNumber("Wrist/Tuning/kA", ClawConstants.Wrist.kA);
    mClawFeedforward = new ArmFeedforward(0.0, kG, kV, kA);
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mClawFeedforward.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    double calculatedOutput = mClawFeedforward.calculate(Math.toRadians(getMeasurement()), 0);

    mClawSubsystem.setWrist(calculatedOutput);

    SmartDashboard.putNumber("Wrist/FF Calculation", calculatedOutput);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mClawFeedforward.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double getMeasurement() {
    return mClawSubsystem.getEncoderMeasurement();
  }
}
