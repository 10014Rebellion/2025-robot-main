package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.ConfigMotor;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax mElevatorMotor =
      new SparkMax(ElevatorConstants.kMotorId, MotorType.kBrushless);
  private final RelativeEncoder mEncoder = mElevatorMotor.getEncoder();

  public ElevatorIOSpark() {
    ConfigMotor.configSparkMax(
        mElevatorMotor,
        ElevatorConstants.kInverted,
        ElevatorConstants.kCurrentLimit,
        ElevatorConstants.kIdleMode,
        mEncoder,
        ElevatorConstants.kEncConversionFactor,
        ElevatorConstants.kEncCPR);
  }

  @Override
  public void updateInputs(ElevatorIOInputs pInputs) {
    SparkUtil.ifOk(mElevatorMotor, mEncoder::getPosition, (value) -> pInputs.mPosition = value);
    SparkUtil.ifOk(mElevatorMotor, mEncoder::getVelocity, (value) -> pInputs.mVelocity = value);
    SparkUtil.ifOk(
        mElevatorMotor,
        new DoubleSupplier[] {mElevatorMotor::getAppliedOutput, mElevatorMotor::getBusVoltage},
        (values) -> pInputs.mAppliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(
        mElevatorMotor,
        mElevatorMotor::getOutputCurrent,
        (value) -> pInputs.mAppliedCurrent = value);
  }

  @Override
  public void setVoltage(double pVoltage) {
    mElevatorMotor.setVoltage(pVoltage);
  }

  @Override
  public void setPercentOutput(double pSpeed) {
    mElevatorMotor.set(pSpeed);
  }

  @Override
  public void disable() {
    mElevatorMotor.disable();
  }

  @Override
  public double getExtension() {
    return mEncoder.getPosition();
  }
}
