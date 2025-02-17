package frc.robot.subsystems.elevatorPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorPivot.ElevatorPivotConstants.Positions;

public class ElevatorPivotPIDCommand extends Command {
  private final ElevatorPivot mPivot;
  private final PIDController mPIDController;
  private double mSetpoint;
  private boolean IS_TUNING = true;

  public ElevatorPivotPIDCommand(Positions pSetpoint, ElevatorPivot pElevatorPivotSubsystem) {
    this(false, pSetpoint.getPos(), pElevatorPivotSubsystem);
  }

  public ElevatorPivotPIDCommand(
      boolean isTuning, double pSetpoint, ElevatorPivot pElevatorPivotSubsystem) {
    this.mPivot = pElevatorPivotSubsystem;
    this.IS_TUNING = isTuning;

    this.mPIDController =
        new PIDController(ElevatorPivotConstants.kP, 0.0, ElevatorPivotConstants.kD);
    this.mPIDController.setTolerance(ElevatorPivotConstants.kTolerance);

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
              ElevatorPivotConstants.kReverseSoftLimit,
              ElevatorPivotConstants.kForwardSoftLimit);
    }

    addRequirements(pElevatorPivotSubsystem);
  }

  @Override
  public void initialize() {
    double kP = SmartDashboard.getNumber("TunableNumbers/Pivot/kP", 0);
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
