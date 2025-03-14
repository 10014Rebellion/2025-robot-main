package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.PivotConstants.Positions;

public class PivotPIDCommand extends Command {
  private final Pivot mPivot;
  private final PIDController mPIDController;
  private double mSetpoint;
  private boolean IS_TUNING = true;

  public PivotPIDCommand(Positions pSetpoint, Pivot pPivotSubsystem) {
    this(false, pSetpoint.getPos(), pPivotSubsystem);
  }

  public PivotPIDCommand(
      boolean isTuning, double pSetpoint, Pivot pPivotSubsystem) {
    this.mPivot = pPivotSubsystem;
    this.IS_TUNING = isTuning;

    this.mPIDController =
        new PIDController(PivotConstants.kP, 0.0, PivotConstants.kD);
    this.mPIDController.setTolerance(PivotConstants.kTolerance);

    if (IS_TUNING) {
      this.mSetpoint = SmartDashboard.getNumber("TunableNumbers/Pivot/Setpoint", 0);
      System.out.println(
          String.format(
              "<<< %s - %s is in TUNING mode. >>>\n",
              this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
    } else {
      this.mSetpoint =
          MathUtil.clamp(
              pSetpoint,
              PivotConstants.kReverseSoftLimit,
              PivotConstants.kForwardSoftLimit);
    }

    addRequirements(pPivotSubsystem);
  }

  @Override
  public void initialize() {
    double kP = SmartDashboard.getNumber("TunableNumbers/Pivot/kP", 0);
    mPIDController.setPID(kP, 0, PivotConstants.kD);
    mPIDController.reset();
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    if (IS_TUNING) {
      mSetpoint = SmartDashboard.getNumber("TunableNumbers/Pivot/Tunable Setpoint", 0);
    }

    SmartDashboard.putNumber("Wrist/Setpoint", mSetpoint);
    double calculatedOutput = mPIDController.calculate(getMeasurement(), mSetpoint);
    mPivot.setVoltage(calculatedOutput);
    SmartDashboard.putNumber("Wrist/Output Value", calculatedOutput);
  }

  @Override
  public void end(boolean interrupted) {
    mPivot.setVoltage(0);
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
    return mPivot.getEncoderMeasurement();
  }
}
