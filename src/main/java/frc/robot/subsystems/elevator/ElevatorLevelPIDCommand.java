package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorLevelPIDCommand extends Command {
  private final Elevator mElevatorSubsystem;
  private double mSetpoint;
  private final ProfiledPIDController mProfiledPIDController;
  private final ElevatorFeedforward mElevatorFeedforward;

  public ElevatorLevelPIDCommand(Elevator pElevatorSubsystem) {
    this.mElevatorSubsystem = pElevatorSubsystem;
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

    double pSetpoint =
        SmartDashboard.getNumber("Levels/Elevator Setpoint", ElevatorConstants.Positions.L2.getPos());
    this.mSetpoint =
        MathUtil.clamp(
            pSetpoint, ElevatorConstants.kReverseSoftLimit, ElevatorConstants.kForwardSoftLimit);

    SmartDashboard.putNumber("Elevator/PID Output", 0.0);
    addRequirements(pElevatorSubsystem);
  }

  @Override
  public void initialize() {
    mSetpoint =
        SmartDashboard.getNumber(
            "Levels/Elevator Setpoint", ElevatorConstants.Positions.L2.getPos());
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
    mSetpoint =
        SmartDashboard.getNumber(
            "Levels/Elevator Setpoint", ElevatorConstants.Positions.L2.getPos());

    double calculatedFeedforward = mElevatorFeedforward.calculate(0);
    double calculatedProfilePID = mProfiledPIDController.calculate(getMeasurement(), mSetpoint);
    double calculatedOutput = calculatedFeedforward + calculatedProfilePID;
    mElevatorSubsystem.setMotorVoltage(calculatedOutput);

    SmartDashboard.putNumber("Elevator/Calculated Output", calculatedOutput);
    SmartDashboard.putNumber("Elevator/Setpoint", mSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    mElevatorSubsystem.setMotorVoltage(mElevatorFeedforward.calculate(0.0));
    System.out.println(
        String.format(
            "<<< %s - %s is ENDING :C >>>\n",
            this.getClass().getSimpleName(), mProfiledPIDController.getClass().getSimpleName()));
  }

  @Override
  public boolean isFinished() {
    return mProfiledPIDController.atGoal();
  }

  private double getMeasurement() {
    return mElevatorSubsystem.getEncoderMeasurement();
  }
}
