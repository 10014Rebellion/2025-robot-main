package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax mClimbMotor;
  private final SparkMax mGrabberMotor;

  public ClimbSubsystem() {
    mClimbMotor = new SparkMax(ClimbConstants.Pulley.kMotorID, ClimbConstants.Pulley.kMotorType);
    mClimbMotor.configure(
        ClimbConstants.Pulley.kClimbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mGrabberMotor =
        new SparkMax(ClimbConstants.Grabber.kMotorID, ClimbConstants.Grabber.kMotorType);
    mGrabberMotor.configure(
        ClimbConstants.Grabber.kClimbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public FunctionalCommand setGrabberVoltsCmd(ClimbConstants.Grabber.VoltageSetpoints pVolts) {
    return setGrabberVoltsCmd(pVolts.getVolts());
  }

  public FunctionalCommand setGrabberVoltsCmd(double pVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setGrabberVolts(pVolts);
        },
        (interrupted) -> setGrabberVolts(0),
        () -> false,
        this);
  }

  private void setGrabberVolts(double pVolts) {
    mGrabberMotor.setVoltage(MathUtil.clamp(pVolts, -12, 12));
  }

  public FunctionalCommand setPulleyVoltsCmd(ClimbConstants.Pulley.VoltageSetpoints pVolts) {
    return setPulleyVoltsCmd(pVolts.getVolts());
  }

  public FunctionalCommand setPulleyVoltsCmd(double pVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setPulleyVolts(pVolts);
        },
        (interrupted) -> setPulleyVolts(0),
        () -> false,
        this);
  }

  private void setPulleyVolts(double pVolts) {
    mClimbMotor.setVoltage(MathUtil.clamp(pVolts, -12, 12));
  }
}
