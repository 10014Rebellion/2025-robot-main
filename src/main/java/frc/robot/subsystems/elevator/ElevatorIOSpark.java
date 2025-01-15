package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.ConfigMotor;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax elevatorMotor =
      new SparkMax(ElevatorConstants.kMotorId, MotorType.kBrushless);
  private final RelativeEncoder encoder = elevatorMotor.getEncoder();

  public ElevatorIOSpark() {
    ConfigMotor.configSparkMax(
        elevatorMotor,
        ElevatorConstants.kInverted,
        ElevatorConstants.kCurrentLimit,
        ElevatorConstants.kIdleMode,
        encoder,
        ElevatorConstants.kEncConversionFactor,
        ElevatorConstants.kEncCPR);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    SparkUtil.ifOk(elevatorMotor, encoder::getPosition, (value) -> inputs.position = value);
    SparkUtil.ifOk(elevatorMotor, encoder::getVelocity, (value) -> inputs.velocity = value);
    SparkUtil.ifOk(
        elevatorMotor,
        new DoubleSupplier[] {elevatorMotor::getAppliedOutput, elevatorMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(
        elevatorMotor, elevatorMotor::getOutputCurrent, (value) -> inputs.appliedCurrent = value);
  }

  @Override
  public void setVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

  @Override
  public void setPercentOutput(double speed) {
    elevatorMotor.set(speed);
  }
}
