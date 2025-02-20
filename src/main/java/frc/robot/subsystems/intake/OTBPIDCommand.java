package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorPivot.ElevatorPivotConstants;
import frc.robot.subsystems.intake.IntakeConstants.OTBIntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.OTBIntakeConstants.Positions;

public class OTBPIDCommand extends Command {
  private final OTBIntake mIntake;
  private final PIDController mPIDController;
  private double mSetpoint;
  private boolean IS_TUNING = true;

  public OTBPIDCommand(Positions pSetpoint, OTBIntake pElevatorPivotSubsystem) {
    this(false, pSetpoint.getPos(), pElevatorPivotSubsystem);
  }

  public OTBPIDCommand(boolean isTuning, double pSetpoint, OTBIntake pOTBIntakeSubsystem) {
    this.mIntake = pOTBIntakeSubsystem;
    this.IS_TUNING = isTuning;

    this.mPIDController = new PIDController(OTBIntakeConstants.kP, 0.0, OTBIntakeConstants.kD);
    this.mPIDController.setTolerance(OTBIntakeConstants.kTolerance);

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
              ElevatorPivotConstants.kReverseSoftLimit,
              ElevatorPivotConstants.kForwardSoftLimit);
    }

    addRequirements(pOTBIntakeSubsystem);
  }

  @Override
  public void initialize() {
    double kP = SmartDashboard.getNumber("TunableNumbers/Intake/kP", 0);
    mPIDController.setPID(kP, 0, ElevatorPivotConstants.kD);
    mPIDController.reset();
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
    double calculatedOutput = mPIDController.calculate(getMeasurement(), mSetpoint);
    mIntake.setRightPivot(calculatedOutput);
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
