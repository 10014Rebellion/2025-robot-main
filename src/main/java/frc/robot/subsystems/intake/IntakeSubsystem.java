package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.OTBIntakeConstants;
import frc.robot.util.TunableNumber;

public class IntakeSubsystem extends SubsystemBase {
  private SparkFlex mRightPivotMotor;
  private SparkFlex mRightRollerMotor;
  private SparkFlex mFunnelMotor;
  private SparkFlex mIndexerMotor;
  private DigitalInput mCoralSensor1;

  private TunableNumber kP, kD, tunableSetpoint;

  private DutyCycleEncoder mRightPivotEncoder;

  public IntakeSubsystem() {
    this.mRightPivotMotor =
        new SparkFlex(OTBIntakeConstants.kRightPivotID, OTBIntakeConstants.kMotorType);
    this.mRightRollerMotor =
        new SparkFlex(OTBIntakeConstants.kRightRollerID, OTBIntakeConstants.kMotorType);
    this.mFunnelMotor =
        new SparkFlex(IntakeConstants.Funnel.kFunnelID, IntakeConstants.Funnel.kMotorType);
    this.mIndexerMotor =
        new SparkFlex(IntakeConstants.Funnel.kIndexerID, IntakeConstants.Funnel.kMotorType);

    mRightPivotMotor.configure(
        OTBIntakeConstants.kRightPivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    mRightRollerMotor.configure(
        OTBIntakeConstants.kRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    mFunnelMotor.configure(
        IntakeConstants.Funnel.kFunnelConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    mIndexerMotor.configure(
        IntakeConstants.Funnel.kIndexerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    mRightPivotEncoder = new DutyCycleEncoder(OTBIntakeConstants.kEncoderDIOPort);
    mRightPivotEncoder.setDutyCycleRange(0, 1);

    kP = new TunableNumber("Intake/kP", OTBIntakeConstants.kP);
    kD = new TunableNumber("Intake/kD", OTBIntakeConstants.kD);
    tunableSetpoint = new TunableNumber("Intake/Tunable Setpoint", 0);
    SmartDashboard.putNumber("Intake/Indexer Volts", 1);
    SmartDashboard.putNumber("Intake/Intake Volts", 1);
    SmartDashboard.putNumber("Intake/kG", IntakeConstants.OTBIntakeConstants.kG);

    mCoralSensor1 = new DigitalInput(OTBIntakeConstants.kSensor1DIOPort);
  }

  public void setRightRoller(double pVoltage) {
    mRightRollerMotor.setVoltage(filterVoltage(pVoltage));
  }

  public void setRightPivot(double pVoltage) {
    if (pVoltage < 1) {
      mRightPivotMotor.setVoltage(filterVoltage(pVoltage * 0.5));
    } else mRightPivotMotor.setVoltage(filterVoltage(pVoltage));
  }

  public void setFunnel(double pVoltage) {
    mFunnelMotor.setVoltage(filterVoltage(pVoltage));
  }

  public void setIndexer(double pVoltage) {
    mIndexerMotor.setVoltage(pVoltage);
  }

  private double filterVoltage(double pVoltage) {
    return MathUtil.clamp(pVoltage, -12.0, 12.0);
    // return MathUtil.clamp(pVoltage, -12.0, 12.0);
  }

  private double filterToLimits(double pInput) {
    return (pInput > 0 && getEncoderMeasurement() >= OTBIntakeConstants.kForwardSoftLimit)
            || (pInput < 0 && getEncoderMeasurement() <= OTBIntakeConstants.kReverseSoftLimit)
        ? 0.0
        : pInput;
  }

  private void stopIfLimit() {
    double motorOutput = getMotorOutput();
    if ((motorOutput > 0 && getEncoderMeasurement() >= OTBIntakeConstants.kForwardSoftLimit)
        || (motorOutput < 0 && getEncoderMeasurement() <= OTBIntakeConstants.kReverseSoftLimit)) {
      mRightPivotMotor.setVoltage(0);
    }
  }

  public double getMotorOutput() {
    return mRightPivotMotor.getAppliedOutput();
  }

  public double getEncoderMeasurement() {
    double measurement =
        -(mRightPivotEncoder.get() * 360.0) + IntakeConstants.OTBIntakeConstants.kEncoderOffset;
    if (measurement >= 180) {
      return measurement - 360;
    }
    return measurement;
  }

  public boolean getCoralDetected() {
    return !mCoralSensor1.get();
  }

  @Override
  public void periodic() {
    // stopIfLimit();
    SmartDashboard.putNumber("Intake/Pivot Position", getEncoderMeasurement());
    SmartDashboard.putNumber("Intake/Pivot Current", mRightPivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Roller Current", mRightRollerMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Intake/Coral Detected", getCoralDetected());
  }
}
