package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants.IntakePositions;
import frc.robot.subsystems.intake.IntakeConstants.OTBIntakeConstants;

public class autoIntakeCoralCommand extends Command {
  private final IntakeSubsystem mIntake;
  private final PIDController mPIDController;
  private ArmFeedforward mFeedforward;
  private double mSetpoint;

  public autoIntakeCoralCommand(IntakeSubsystem intakeSubsytem) {
    this.mIntake = intakeSubsytem;
    this.mPIDController = new PIDController(OTBIntakeConstants.kP, 0.0, OTBIntakeConstants.kD);
    this.mPIDController.setTolerance(OTBIntakeConstants.kTolerance);

    this.mFeedforward = new ArmFeedforward(0, IntakeConstants.OTBIntakeConstants.kG, 0);
    this.mSetpoint =
        MathUtil.clamp(
            IntakePositions.INTAKING.getPos(),
            IntakeConstants.OTBIntakeConstants.kReverseSoftLimit,
            IntakeConstants.OTBIntakeConstants.kForwardSoftLimit);
    addRequirements(intakeSubsytem);
  }

  @Override
  public void initialize() {
    mPIDController.setPID(OTBIntakeConstants.kP, 0, OTBIntakeConstants.kD);
    mPIDController.reset();
    mFeedforward =
        new ArmFeedforward(
            0, SmartDashboard.getNumber("Intake/kG", IntakeConstants.OTBIntakeConstants.kG), 0.0);
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mIntake.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    double calculatedPID = mPIDController.calculate(getMeasurement(), mSetpoint);
    double calculatedFeedforward = mFeedforward.calculate(getMeasurement(), 0.0);
    double calculatedOutput = calculatedPID + calculatedFeedforward;
    mIntake.setRightPivot(calculatedOutput);
    SmartDashboard.putNumber("Intake/FF Output", calculatedFeedforward);
    SmartDashboard.putNumber("Intake/Pivot Output", calculatedOutput);
    mIntake.setFunnel(0.7);
    mIntake.setIndexer(2);
    mIntake.setRightRoller(8);
  }

  @Override
  public void end(boolean interrupted) {
    mIntake.setFunnel(0);
    mIntake.setIndexer(0);
    mIntake.setRightRoller(0);
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :( >>>\n",
            this.getClass().getSimpleName(), mIntake.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return mIntake.getCoralDetected();
  }

  public double getMeasurement() {
    return mIntake.getEncoderMeasurement();
  }
}
