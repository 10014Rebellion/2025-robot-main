package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFFCommand extends Command {

  private final OTBIntake mIntake;
  private ArmFeedforward mIntakeFeedforward;
  private double pivotPosition;

  public IntakeFFCommand(OTBIntake pIntakeSubsystem) {
    this.mIntake = pIntakeSubsystem;
    this.mIntakeFeedforward = new ArmFeedforward(0.0, IntakeConstants.OTBIntakeConstants.kG, 0.0);
    this.pivotPosition = SmartDashboard.getNumber("Pivot/Position", 0.0);
    addRequirements(pIntakeSubsystem);
  }

  @Override
  public void initialize() {
    mIntakeFeedforward =
        new ArmFeedforward(
            0.0, SmartDashboard.getNumber("Intake/kG", IntakeConstants.OTBIntakeConstants.kG), 0.0);
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mIntakeFeedforward.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    double calculatedOutput = mIntakeFeedforward.calculate(getMeasurement(), 0);

    mIntake.setRightPivot(calculatedOutput);

    SmartDashboard.putNumber("Intake/FF Calculation", calculatedOutput);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mIntakeFeedforward.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double getMeasurement() {
    return mIntake.getEncoderMeasurement();
  }
}
