package frc.robot.subsystems.climb;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbConstants.Pulley;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax mClimbMotor;
  private final SparkMax mGrabberMotor;
  private final AbsoluteEncoder mClimbEncoder;
  private final DigitalInput mBeamBreak;

  public ClimbSubsystem() {
    mClimbMotor = new SparkMax(ClimbConstants.Pulley.kMotorID, ClimbConstants.Pulley.kMotorType);
    mBeamBreak = new DigitalInput(ClimbConstants.Grabber.kClimbBeamBreakID);
    mClimbMotor.configure(
        ClimbConstants.Pulley.kClimbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mClimbEncoder = mClimbMotor.getAbsoluteEncoder();
    mGrabberMotor = new SparkMax(ClimbConstants.Grabber.kMotorID, ClimbConstants.Grabber.kMotorType);
    mGrabberMotor.configure(
        ClimbConstants.Grabber.kClimbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  public boolean getBeamBroken() {
    return !mBeamBreak.get();
  }

  public FunctionalCommand setGrabberVoltsCmd(ClimbConstants.Grabber.VoltageSetpoints pVolts) {
    return setGrabberVoltsCmd(pVolts.getVolts());
  }

  public FunctionalCommand setGrabberVoltsCmd(double pVolts) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          setGrabberVolts(pVolts);
        },
        (interrupted) -> setGrabberVolts(0),
        () -> false);
  }

  public void setGrabberVolts(double pVolts) {
    mGrabberMotor.setVoltage(MathUtil.clamp(pVolts, -12, 12));
  }

  public FunctionalCommand setPulleyVoltsCmd(ClimbConstants.Pulley.VoltageSetpoints pVolts) {
    return setPulleyVoltsCmd(pVolts.getVolts());
  }

  public FunctionalCommand setPulleyVoltsCmd(double pVolts) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          setPulleyVolts(pVolts);
        },
        (interrupted) -> setPulleyVolts(0),
        () -> false);
  }

  private void setPulleyVolts(double pVolts) {
    mClimbMotor.setVoltage(MathUtil.clamp(pVolts, -12, 12));
  }

  private void setPulleyVolts(Pulley.VoltageSetpoints pVoltSetpoint) {
    setPulleyVolts(pVoltSetpoint.getVolts());
  }

  private double getEncReading() {
    double encoderMeasurement = mClimbEncoder.getPosition();
    if (encoderMeasurement > 180.0)
      encoderMeasurement -= 360.0;
    return encoderMeasurement;
  }

  public FunctionalCommand climbToSetpoint(Pulley.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          double encReading = getEncReading();
          if (encReading > pSetpoint.getPos()) {
            setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.DESCEND.getVolts());
          }
          if (encReading < pSetpoint.getPos()) {
            setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.ASCEND.getVolts());
          }
        },
        (interrupted) -> {
          setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.STOP);
          setGrabberVolts(ClimbConstants.Grabber.VoltageSetpoints.PULL_IN.getVolts());
        },
        () -> (Math.abs(getEncReading() - pSetpoint.getPos()) < ClimbConstants.Pulley.kTolerance),
        this);
  }

  public boolean getTolerance(Pulley.Setpoints pSetpoint) {
    return (Math.abs(getEncReading() - pSetpoint.getPos()) < ClimbConstants.Pulley.kTolerance);
  }

  public FunctionalCommand climbToSetpointRetracted(Pulley.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          double encReading = getEncReading();
          if (encReading > pSetpoint.getPos()) {
            setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.DESCEND.getVolts());
          }
          if (encReading < pSetpoint.getPos()) {
            setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.STOP.getVolts());
          }
        },
        (interrupted) -> {
          setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.STOP);
          setGrabberVolts(ClimbConstants.Grabber.VoltageSetpoints.PULL_IN.getVolts());
        },
        () -> (Math.abs(getEncReading() - pSetpoint.getPos()) < ClimbConstants.Pulley.kTolerance),
        this);
  }

  public FunctionalCommand climbUntilRetracted() {
    return new FunctionalCommand(
        () -> setPulleyVolts(
            ClimbConstants.Pulley.VoltageSetpoints.ASCEND),
        () -> setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.ASCEND),
        (interrupted) -> setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.STOP),
        () -> getEncReading() >= ClimbConstants.Pulley.Setpoints.CLIMBED.getPos(),
        this);
  }

  public FunctionalCommand retractClimb() {
    return new FunctionalCommand(
        () -> setPulleyVolts(
            ClimbConstants.Pulley.VoltageSetpoints.ASCEND),
        () -> setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.ASCEND),
        (interrupted) -> setPulleyVolts(ClimbConstants.Pulley.VoltageSetpoints.STOP),
        () -> getEncReading() > ClimbConstants.Pulley.Setpoints.STOWED.getPos(),
        this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb/Position", getEncReading());
    SmartDashboard.putBoolean("Climb/BeamBroken", getBeamBroken());
    SmartDashboard.putBoolean(
        "Climb/Fully Extended",
        getEncReading() < ClimbConstants.Pulley.Setpoints.EXTENDED.getPos());
  }
}
