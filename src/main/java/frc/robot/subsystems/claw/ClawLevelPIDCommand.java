package frc.robot.subsystems.claw;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawConstants.Wrist;
import frc.robot.subsystems.claw.ClawConstants.Wrist.Positions;
import java.util.function.Supplier;

public class ClawLevelPIDCommand extends Command {
  private final Claw mClawSubsystem;
  private double mSetpoint;
  private final PIDController mPIDController;
  private final ArmFeedforward mClawFeedforward;
  private double elevatorSetpoint;
  private double pivotPosition;

  public ClawLevelPIDCommand(Claw pClawSubsystem) {
    this.mClawSubsystem = pClawSubsystem;

    this.mPIDController = new PIDController(Wrist.kP, 0.0, Wrist.kD);
    this.mPIDController.setTolerance(Wrist.kTolerance);
    this.mClawFeedforward = new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);

    double pSetpoint = SmartDashboard.getNumber("Levels/Wrist Setpoint", ClawConstants.Wrist.Positions.L2.getPos());
    this.mSetpoint = MathUtil.clamp(pSetpoint, Wrist.kReverseSoftLimit, Wrist.kForwardSoftLimit);


    this.elevatorSetpoint = SmartDashboard.getNumber("Elevator/Setpoint", 0);
    this.pivotPosition = SmartDashboard.getNumber("Pivot/Position", 0.0);
    SmartDashboard.putNumber("Tuning/Wrist/Output Value", 0.0);
    SmartDashboard.putNumber("Wrist/Setpoint", pSetpoint);

    addRequirements(pClawSubsystem);
  }

  @Override
  public void initialize() {
    mPIDController.setPID(Wrist.kP, 0, Wrist.kD);
    // mPIDController.setConstraints(
    //     new TrapezoidProfile.Constraints(Wrist.kMaxVelocity, Wrist.kMaxAcceleration));
    // mPIDController.reset(getMeasurement());
    mPIDController.reset();
    System.out.println(
        String.format(
            "<<< %s - %s is STARTING :D >>>\n",
            this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
  }

  @Override
  public void execute() {
    mSetpoint = SmartDashboard.getNumber("Levels/Wrist Setpoint", ClawConstants.Wrist.Positions.L2.getPos());
    elevatorSetpoint = SmartDashboard.getNumber("Elevator/Setpoint", 0);
    if ((elevatorSetpoint <= 5) && (mSetpoint < 10)) {
      mSetpoint = 10;
      System.out.println(
          String.format(
              "<<< %s - %s is going too far down! >>>\n",
              this.getClass().getSimpleName(), mPIDController.getClass().getSimpleName()));
    }
    SmartDashboard.putNumber("Wrist/Setpoint", mSetpoint);
    double calculatedFeedforward = mClawFeedforward.calculate(getMeasurement() + pivotPosition, 0.0);
    double calculatedProfilePID = mPIDController.calculate(getMeasurement() + pivotPosition, mSetpoint);
    double calculatedOutput = calculatedProfilePID + calculatedFeedforward;
    mClawSubsystem.setWrist(calculatedOutput);
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
