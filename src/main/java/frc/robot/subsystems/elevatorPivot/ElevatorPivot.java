package frc.robot.subsystems.elevatorPivot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

public class ElevatorPivot extends SubsystemBase {
  private final SparkMax mPivotMotor;
  private final TunableNumber mTunableVoltage;

  public ElevatorPivot() {
    this.mPivotMotor =
        new SparkMax(ElevatorPivotConstants.kMotorID, ElevatorPivotConstants.kMotorType);
    mPivotMotor.configure(
        ElevatorPivotConstants.kPivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    mTunableVoltage = new TunableNumber("Tuning/Pivot/Setvolts", 0);
  }

  public void setVoltage(double pVoltage) {
    mPivotMotor.setVoltage(filterVoltage(pVoltage));
  }

  public void setVoltageTunable() {
    setVoltage(SmartDashboard.getNumber("TunableNumbers/Tuning/Pivot/Setvolts", 0));
  }

  private double filterVoltage(double pVoltage) {
    return (MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Voltage", mPivotMotor.getBusVoltage());
  }
}
