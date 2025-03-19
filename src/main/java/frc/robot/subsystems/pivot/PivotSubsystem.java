package frc.robot.subsystems.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  private final SparkMax mPivotMotor;
  private final AbsoluteEncoder mPivotEncoder;
  private final ArmFeedforward mPivotFF;
  private final ProfiledPIDController mPivotProfiledPID;

  private Controllers mCurrentController;

  private enum Controllers {
    ProfiledPID,
    Feedforward,
    Manual
  }

  public PivotSubsystem() {
    this.mPivotMotor = new SparkMax(PivotConstants.kMotorID, PivotConstants.kMotorType);
    this.mPivotProfiledPID =
        new ProfiledPIDController(
            PivotConstants.kP,
            0,
            PivotConstants.kD,
            new Constraints(PivotConstants.kMaxVelocity, PivotConstants.kMaxAcceleration));
    this.mPivotFF =
        new ArmFeedforward(
            PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);
    this.mPivotEncoder = mPivotMotor.getAbsoluteEncoder();
    mPivotMotor.configure(
        PivotConstants.kPivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public boolean isPIDAtGoal() {
    return mCurrentController.equals(Controllers.ProfiledPID) && mPivotProfiledPID.atGoal();
  }

  public FunctionalCommand setPIDCmd(PivotConstants.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.ProfiledPID;
          mPivotProfiledPID.setGoal(pSetpoint.getPos());
        },
        () -> {
          double encoderReading = getEncReading();
          double calculatedPID = mPivotFF.calculate(encoderReading, 0.0);
          double calculatedFF = mPivotProfiledPID.calculate(encoderReading);
          setVolts(calculatedPID + calculatedFF);
        },
        (interrupted) -> setVolts(0),
        () -> isPIDAtGoal(),
        this);
  }

  public InstantCommand setVoltsCmd(double pVoltage) {
    return new InstantCommand(
        () -> {
          mCurrentController = Controllers.Manual;
          setVolts(pVoltage);
        });
  }

  public InstantCommand forceSetVoltsCmd(double pVoltage) {
    return new InstantCommand(() -> mPivotMotor.setVoltage(pVoltage));
  }

  public void setVolts(double pVoltage) {
    mPivotMotor.setVoltage(filterVoltage(pVoltage));
  }

  public InstantCommand stopCommand() {
    return new InstantCommand(() -> setVolts(0));
  }

  private double filterVoltage(double pVoltage) {
    return (MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getEncReading() {
    double encoderMeasurement = getRawEncReading() * PivotConstants.kPositionConversionFactor;
    if (encoderMeasurement > 180) {
      encoderMeasurement -= 360;
    }
    return encoderMeasurement;
  }

  public double getRawEncReading() {
    return mPivotEncoder.getPosition();
  }

  // private boolean isOutOfBounds(double pInput) {
  //   return (pInput > 0 && getEncReading() >= PivotConstants.kForwardSoftLimit)
  //       || (pInput < 0 && getEncReading() <= PivotConstants.kReverseSoftLimit);
  // }

  // private double filterToLimits(double pInput) {
  //   return isOutOfBounds(pInput) ? 0.0 : pInput;
  // }

  // private void stopIfLimit() {
  //   if (isOutOfBounds(getMotorOutput())) {
  //     setVolts(0);
  //   }
  // }

  public double getMotorOutput() {
    return mPivotMotor.getAppliedOutput();
  }

  @Override
  public void periodic() {
    // stopIfLimit();
    SmartDashboard.putNumber("Pivot/Voltage", mPivotMotor.getBusVoltage());
    SmartDashboard.putNumber("Pivot/Position", getEncReading());
  }
}
