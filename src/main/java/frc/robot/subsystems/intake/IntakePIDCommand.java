package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants.IntakePositions;
import frc.robot.subsystems.intake.IntakeConstants.OTBIntakeConstants;

public class IntakePIDCommand extends Command {
  private final OTBIntake mIntake;
  private final PIDController mPIDController;
  private ArmFeedforward mFeedforward;
  private double mSetpoint;
  private boolean IS_TUNING = true;

  public IntakePIDCommand(IntakePositions pSetpoint, OTBIntake pIntakeSubsystem) {
    this(false, pSetpoint.getPos(), pIntakeSubsystem);
  }

  public IntakePIDCommand(boolean isTuning, double pSetpoint, OTBIntake pIntakeSubsystem) {
    this.mIntake = pIntakeSubsystem;
    this.IS_TUNING = isTuning;

    this.mPIDController = new PIDController(OTBIntakeConstants.kP, 0.0, OTBIntakeConstants.kD);
    this.mPIDController.setTolerance(OTBIntakeConstants.kTolerance);

    this.mFeedforward = new ArmFeedforward(0, IntakeConstants.OTBIntakeConstants.kG, 0);

    if (IS_TUNING) {
      this.mSetpoint = SmartDashboard.getNumber("TunableNumbers/Intake/Tunable Setpoint", 0);
      System.out.println(
          String.format(
              "<<< %s - %s is in TUNING mode. >>>\n",
              this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
    } else {
      this.mSetpoint =
          MathUtil.clamp(
              pSetpoint,
              IntakeConstants.OTBIntakeConstants.kReverseSoftLimit,
              IntakeConstants.OTBIntakeConstants.kForwardSoftLimit);
    }

    addRequirements(pIntakeSubsystem);
  }

  @Override
  public void initialize() {
    double kP = SmartDashboard.getNumber("TunableNumbers/Intake/kP", 0);
    mPIDController.setPID(kP, 0, 0.0);
    mPIDController.reset();
    mFeedforward =
        new ArmFeedforward(
            0, SmartDashboard.getNumber("Intake/kG", IntakeConstants.OTBIntakeConstants.kG), 0.0);
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    if (IS_TUNING) {
      mSetpoint = SmartDashboard.getNumber("TunableNumbers/Intake/Tunable Setpoint", 0);
    }

    SmartDashboard.putNumber("Intake/Setpoint", mSetpoint);
    double calculatedPID = mPIDController.calculate(getMeasurement(), mSetpoint);
    double calculatedFeedforward = mFeedforward.calculate(getMeasurement(), 0.0);
    double calculatedOutput = calculatedPID + calculatedFeedforward;
    mIntake.setRightPivot(calculatedOutput);
    SmartDashboard.putNumber("Intake/FF Output", calculatedFeedforward);
    SmartDashboard.putNumber("Intake/Pivot Output", calculatedOutput);
  }

  @Override
  public void end(boolean interrupted) {
    mIntake.setRightPivot(0);
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return false; // mProfiledPIDController.atSetpoint();
  }

  private double getMeasurement() {
    return mIntake.getEncoderMeasurement();
  }
}
