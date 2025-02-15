package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkFlex mFunnelMotor;

  public Intake() {
    this.mFunnelMotor =
        new SparkFlex(IntakeConstants.Funnel.kMotorID, IntakeConstants.Funnel.kMotorType);
    mFunnelMotor.configure(
        IntakeConstants.Funnel.kFunnelConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void setFunnelVoltage(double pVoltage) {
    mFunnelMotor.setVoltage(filterVoltage(pVoltage));
  }

  private double filterVoltage(double pVoltage) {
    return (MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Funnel Voltage", mFunnelMotor.getBusVoltage());
  }
}
