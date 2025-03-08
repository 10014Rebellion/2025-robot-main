package frc.robot.subsystems.elevatorPivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.util.TunableNumber;

public class ElevatorPivot extends SubsystemBase {
  private final SparkMax mPivotMotor;
  private final AbsoluteEncoder mPivotEncoder;
  private final TunableNumber kP;

  public ElevatorPivot() {
    this.mPivotMotor =
        new SparkMax(ElevatorPivotConstants.kMotorID, ElevatorPivotConstants.kMotorType);
    this.mPivotEncoder = mPivotMotor.getAbsoluteEncoder();
    mPivotMotor.configure(
        ElevatorPivotConstants.kPivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    kP = new TunableNumber("Pivot/kP", ElevatorPivotConstants.kP);
  }

  public void setVoltage(double pVoltage) {
    mPivotMotor.setVoltage(filterVoltage(pVoltage));
  }

  public InstantCommand stopCommand() {
    return new InstantCommand(() -> setVoltage(0));
  }

  // public void setVoltageTunable() {
  //   setVoltage(SmartDashboard.getNumber("TunableNumbers/Tuning/Pivot/Setvolts", 0));
  // }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getEncoderMeasurement() {
    double encoderMeasurement = mPivotEncoder.getPosition();
    return encoderMeasurement;
  }

  private double filterToLimits(double pInput) {
    return (pInput > 0 && getEncoderMeasurement() >= ElevatorPivotConstants.kForwardSoftLimit)
            || (pInput < 0 && getEncoderMeasurement() <= ElevatorPivotConstants.kReverseSoftLimit)
        ? 0.0
        : pInput;
  }

  private void stopIfLimit() {
    double motorOutput = getMotorOutput();
    if ((motorOutput > 0 && getEncoderMeasurement() >= ClawConstants.Wrist.kForwardSoftLimit)
        || (motorOutput < 0 && getEncoderMeasurement() <= ClawConstants.Wrist.kReverseSoftLimit)) {
      setVoltage(0);
    }
  }

  public double getMotorOutput() {
    return mPivotMotor.getAppliedOutput();
  }

  @Override
  public void periodic() {
    stopIfLimit();
    SmartDashboard.putNumber("Pivot/Voltage", mPivotMotor.getBusVoltage());
    SmartDashboard.putNumber(
        "Pivot/Position",
        mPivotEncoder.getPosition() * ElevatorPivotConstants.kPositionConversionFactor);
  }
}
