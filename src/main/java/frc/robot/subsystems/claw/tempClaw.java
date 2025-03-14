package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawConstants.Wrist;
import frc.robot.util.TunableNumber;

public class tempClaw extends SubsystemBase {
  private final SparkFlex mWristSparkFlex;
  private final SparkFlex mClawSparkFlex;

  private final SparkClosedLoopController mWristController;
  // private AbsoluteEncoder mWristEncoder;

  private final DutyCycleEncoder mWristEncoder;

  private TunableNumber tunablePosition;

  private double previousPosition, previousTime, previousVelocity;
  private double velocity, acceleration;

  public tempClaw() {
    this.mClawSparkFlex =
        new SparkFlex(ClawConstants.Claw.kLeftClawID, ClawConstants.Claw.kMotorType);
    this.mWristEncoder = new DutyCycleEncoder(ClawConstants.Wrist.kEncoderDIOPort);

    this.mWristSparkFlex = new SparkFlex(Wrist.kMotorID, Wrist.kMotorType);
    this.mWristController = mWristSparkFlex.getClosedLoopController();
    // this.mWristEncoder = mWristSparkMax.getAbsoluteEncoder();

    mWristSparkFlex.configure(
        Wrist.kWristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    mClawSparkFlex.configure(
        ClawConstants.Claw.kClawConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    SmartDashboard.putNumber("Wrist/Tuning/kP", ClawConstants.Wrist.kP);
    SmartDashboard.putNumber("Wrist/Tuning/kD", ClawConstants.Wrist.kD);
    SmartDashboard.putNumber("Wrist/Tuning/kG", ClawConstants.Wrist.kG);
    SmartDashboard.putNumber("Wrist/Tuning/kV", ClawConstants.Wrist.kV);
    SmartDashboard.putNumber("Wrist/Tuning/kA", ClawConstants.Wrist.kA);
    // tunablePosition = new TunableNumber("Wrist/Tunable Setpoint", 0);
    // wristP.setDefault(0.0);
    // wristD.setDefault(0.0);
    // wristV.setDefault(0.0);
    // wristA.setDefault(0.0);
  }

  public void setWrist(double pVoltage) {
    mWristSparkFlex.setVoltage(-pVoltage);
  }

  public void setClaw(double pVoltage) {
    mClawSparkFlex.setVoltage(-pVoltage);
  }

  public void updateVelocity() {
    double currentTime = Timer.getFPGATimestamp();
    double currentPosition = getEncoderMeasurement();

    // Calculate velocity: ΔPosition / ΔTime
    double deltaPosition = currentPosition - previousPosition;
    double deltaTime = currentTime - previousTime;

    if (deltaTime > 0) {
      velocity = deltaPosition / deltaTime; // Velocity in terms of revolutions per second (rps)
    }

    // Calculate acceleration: ΔVelocity / ΔTime
    double deltaVelocity = velocity - previousVelocity;
    if (deltaTime > 0) {
      acceleration = deltaVelocity / deltaTime; // Acceleration in terms of rps^2
    }

    // Update previous values for the next cycle
    previousPosition = currentPosition;
    previousTime = currentTime;
    previousVelocity = velocity;
  }

  public double getVelocity() {
    return velocity;
  }

  public double getEncoderMeasurement() {
    double encoderMeasurement = -(mWristEncoder.get() * 360) + ClawConstants.Wrist.kEncoderOffset;
    if (encoderMeasurement > 180) encoderMeasurement -= 360;
    return encoderMeasurement;
  }

  @Override
  public void periodic() {
    updateVelocity();
    SmartDashboard.putNumber("Wrist/Position", getEncoderMeasurement());
    SmartDashboard.putNumber("Wrist/Velocity", getVelocity());
    SmartDashboard.putNumber("Wrist/Applied Output", mWristSparkFlex.getAppliedOutput());
  }
}
