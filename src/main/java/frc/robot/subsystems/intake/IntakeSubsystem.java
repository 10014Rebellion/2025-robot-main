package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.Beambreak;
import frc.robot.subsystems.intake.IntakeConstants.Indexer;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivot;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRoller;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.util.*;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax mIntakePivotMotor;
  private final AbsoluteEncoder mEncoder;
  private final SparkFlex mIntakeRollerMotor;
  private final SparkFlex mIndexerMotor;

  // private final DigitalInput mCoralSensor1;
  private final DigitalInput mCoralSensorFront;
  // private final DigitalInput mCoralSensorBack;

  private Controllers mCurrentController;
  private final ProfiledPIDController mIntakePivotProfiledPID;
  private ArmFeedforward mIntakePivotFF;

  private enum Controllers {
    ProfiledPID,
    Feedforward,
    Manual
  }

  public IntakeSubsystem() {
    this.mIntakePivotMotor = new SparkMax(IntakePivot.kPivotID, IntakePivot.kMotorType);
    this.mEncoder = mIntakePivotMotor.getAbsoluteEncoder();
    // this.mEncoder.
    this.mIntakeRollerMotor = new SparkFlex(IntakeRoller.kRollerID, IntakeRoller.kMotorType);
    this.mIndexerMotor = new SparkFlex(Indexer.kIndexerID, Indexer.kMotorType);
    this.mIntakePivotProfiledPID =
        new ProfiledPIDController(
            IntakePivot.kP,
            0,
            IntakePivot.kD,
            new Constraints(IntakePivot.kMaxVelocity, IntakePivot.kMaxAcceleration));
    this.mIntakePivotProfiledPID.setTolerance(IntakeConstants.IntakePivot.kTolerance);
    this.mIntakePivotFF =
        new ArmFeedforward(IntakePivot.kS, IntakePivot.kG, IntakePivot.kV, IntakePivot.kA);

    mIntakePivotMotor.configure(
        IntakePivot.kPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    mIntakeRollerMotor.configure(
        IntakeRoller.kRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    mIndexerMotor.configure(
        Indexer.kIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    this.setDefaultCommand(enableFFCmd());

    SmartDashboard.putNumber("Intake/Indexer Volts", 1);
    SmartDashboard.putNumber("Intake/Intake Volts", 1);
    SmartDashboard.putNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);

    SmartDashboard.putNumber("Intake/PivotKp", IntakeConstants.IntakePivot.kP);
    SmartDashboard.putNumber("Intake/PivotKd", IntakeConstants.IntakePivot.kD);

    SmartDashboard.putNumber("Intake/Tunable Setpoint", 0.0);
    SmartDashboard.putNumber("Intake/Tunable Pivot Voltage", 0.0);

    mCoralSensorFront = new DigitalInput(Beambreak.kFrontSensorDIOPort);
    // mCoralSensorBack = new DigitalInput(Beambreak.kBackSensorDIOPort);
  }

  public FunctionalCommand enableFFCmd() {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.Feedforward;
          double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);
          mIntakePivotFF =
              new ArmFeedforward(
                  IntakeConstants.IntakePivot.kS,
                  newKg,
                  IntakeConstants.IntakePivot.kV,
                  IntakeConstants.IntakePivot.kA);
        },
        () -> {
          double calculatedOutput =
              mIntakePivotFF.calculate(Math.toRadians(getEncoderReading() - 10.0), 0.0);
          setVoltsIntakePivot(calculatedOutput);
        },
        (interrupted) -> setVoltsIntakePivot(0),
        () -> false,
        this);
  }

  public boolean isPIDAtGoalIntakePivot() {
    return mIntakePivotProfiledPID.atGoal();
  }

  public FunctionalCommand setPIDIntakePivotCmd(IntakePivot.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.ProfiledPID;
          double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakeConstants.IntakePivot.kP);
          double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakeConstants.IntakePivot.kD);
          double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);
          mIntakePivotFF =
              new ArmFeedforward(
                  IntakeConstants.IntakePivot.kS,
                  newKg,
                  IntakeConstants.IntakePivot.kV,
                  IntakeConstants.IntakePivot.kA);
          mIntakePivotProfiledPID.setPID(newKp, 0.0, newKd);
          mIntakePivotProfiledPID.reset(getEncoderReading());
          mIntakePivotProfiledPID.setGoal(pSetpoint.getPos());
        },
        () -> {
          double encoderReading = getEncoderReading();
          double calculatedFF =
              mIntakePivotFF.calculate(
                  Math.toRadians(mIntakePivotProfiledPID.getSetpoint().position - 10.0),
                  Math.toRadians(mIntakePivotProfiledPID.getSetpoint().velocity));
          double calculatedPID = mIntakePivotProfiledPID.calculate(encoderReading);
          setVoltsIntakePivot(calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/PID Output", calculatedPID);
          SmartDashboard.putNumber("Intake/FF Output", calculatedFF);
        },
        (interrupted) ->
            setVoltsIntakePivot(mIntakePivotFF.calculate(Math.toRadians(getEncoderReading()), 0.0)),
        () -> false, // isPIDAtGoalIntakePivot(),
        this);
  }

  public FunctionalCommand setEndablePIDIntakePivotCmd(IntakePivot.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.ProfiledPID;
          double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakeConstants.IntakePivot.kP);
          double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakeConstants.IntakePivot.kD);
          double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);
          mIntakePivotFF =
              new ArmFeedforward(
                  IntakeConstants.IntakePivot.kS,
                  newKg,
                  IntakeConstants.IntakePivot.kV,
                  IntakeConstants.IntakePivot.kA);
          mIntakePivotProfiledPID.setPID(newKp, 0.0, newKd);
          mIntakePivotProfiledPID.reset(getEncoderReading());
          mIntakePivotProfiledPID.setGoal(pSetpoint.getPos());
        },
        () -> {
          double encoderReading = getEncoderReading();
          double calculatedFF =
              mIntakePivotFF.calculate(
                  Math.toRadians(mIntakePivotProfiledPID.getSetpoint().position - 10.0),
                  Math.toRadians(mIntakePivotProfiledPID.getSetpoint().velocity));
          double calculatedPID = mIntakePivotProfiledPID.calculate(encoderReading);
          setVoltsIntakePivot(calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/PID Output", calculatedPID);
          SmartDashboard.putNumber("Intake/FF Output", calculatedFF);
        },
        (interrupted) ->
            setVoltsIntakePivot(mIntakePivotFF.calculate(Math.toRadians(getEncoderReading()), 0.0)),
        () -> isPIDAtGoalIntakePivot(),
        this);
  }

  public FunctionalCommand setTunablePIDIntakeCommand() {
    return new FunctionalCommand(
        () -> {
          mCurrentController = Controllers.ProfiledPID;
          double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakeConstants.IntakePivot.kP);
          double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakeConstants.IntakePivot.kD);
          double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);
          double pSetpoint = SmartDashboard.getNumber("Intake/Tunable Setpoint", 0.0);
          mIntakePivotFF =
              new ArmFeedforward(
                  IntakeConstants.IntakePivot.kS,
                  newKg,
                  IntakeConstants.IntakePivot.kV,
                  IntakeConstants.IntakePivot.kA);
          mIntakePivotProfiledPID.setPID(newKp, 0.0, newKd);
          mIntakePivotProfiledPID.reset(getEncoderReading());
          mIntakePivotProfiledPID.setGoal(pSetpoint);
        },
        () -> {
          double encoderReading = getEncoderReading();
          double calculatedFF =
              mIntakePivotFF.calculate(
                  Math.toRadians(mIntakePivotProfiledPID.getSetpoint().position),
                  Math.toRadians(mIntakePivotProfiledPID.getSetpoint().velocity));
          double calculatedPID = mIntakePivotProfiledPID.calculate(encoderReading);
          setVoltsIntakePivot(calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/PID Output", calculatedPID);
          SmartDashboard.putNumber("Intake/FF Output", calculatedFF);
        },
        (interrupted) -> setVoltsIntakePivot(0),
        () -> false, // isPIDAtGoalIntakePivot(),
        this);
  }

  public InstantCommand setVoltsIntakePivotCmd(double pVoltage) {
    return new InstantCommand(
        () -> {
          mCurrentController = Controllers.Manual;
          setVoltsIntakePivot(pVoltage);
        });
  }

  public void setVoltsIntakeRoller(double pVoltage) {
    mIntakeRollerMotor.setVoltage(MathUtil.clamp(pVoltage, -12, 12));
  }

  public void setVoltsIntakePivot(double pVoltage) {
    mIntakePivotMotor.setVoltage(filterVoltageIntakePivot(pVoltage));
  }

  public void setVoltsIndexer(double pVoltage) {
    mIndexerMotor.setVoltage(pVoltage);
  }

  public FunctionalCommand setIndexCoralCmd() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setVoltsIndexer(IntakeConstants.Indexer.kIntakeVolts);
        },
        (interrupted) -> {
          setVoltsIndexer(1.0);
        },
        () -> {
          return getCoralDetected();
        });
  }

  private double filterVoltageIntakePivot(double pVoltage) {
    return filterToLimitsIntakePivot(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  private boolean isOutOfBoundsIntakePivot(double pInput) {
    return (pInput > 0 && getEncoderReading() >= WristConstants.kForwardSoftLimit)
        || (pInput < 0 && getEncoderReading() <= WristConstants.kReverseSoftLimit);
  }

  private double filterToLimitsIntakePivot(double pInput) {
    return isOutOfBoundsIntakePivot(pInput) ? 0.0 : pInput;
  }

  public void stopIfLimitIntakePivot() {
    if (isOutOfBoundsIntakePivot(getMotorOutput())) {
      setVoltsIntakePivot(0);
    }
  }

  public double getMotorOutput() {
    return mIntakePivotMotor.getAppliedOutput();
  }

  public double getEncoderReading() {
    double measurement = mEncoder.getPosition();
    if (measurement >= 180) {
      return measurement - 360;
    }
    return measurement;
  }

  public boolean getCoralDetectedFront() {
    return !mCoralSensorFront.get();
  }

  // public boolean getCoralDetectedBack() {
  //   return !mCoralSensorBack.get();
  // }

  public boolean getCoralDetected() {
    return getCoralDetectedFront(); // && getCoralDetectedBack();
  }

  public FunctionalCommand setPivotCmd(double pVoltage) {
    return new FunctionalCommand(
        () -> setVoltsIntakePivot(pVoltage),
        () -> setVoltsIntakePivot(pVoltage),
        (interrupted) -> setVoltsIntakePivot(0.0),
        () -> false);
  }

  public FunctionalCommand setTunablePivotCmd() {
    return new FunctionalCommand(
        () -> {},
        () ->
            setVoltsIntakePivot(
                SmartDashboard.getNumber("Intake/Tunable Pivot Voltage", 0.0)
                    + IntakeConstants.IntakePivot.kS),
        (interrupted) -> setVoltsIntakePivot(0.0),
        () -> false,
        this);
  }

  public FunctionalCommand setRollerCmd(double pVoltage) {
    return new FunctionalCommand(
        () -> setVoltsIntakeRoller(pVoltage),
        () -> setVoltsIntakeRoller(pVoltage),
        (interrupted) -> setVoltsIntakeRoller(0.0),
        () -> false);
  }

  public InstantCommand setIndexerCmd(double pVoltage) {
    return new InstantCommand(() -> setVoltsIndexer(pVoltage));
  }

  @Override
  public void periodic() {
    stopIfLimitIntakePivot();
    SmartDashboard.putNumber("Intake/Pivot Position", getEncoderReading());
    SmartDashboard.putNumber("Intake/Pivot Current", mIntakePivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Pivot Voltage", mIntakePivotMotor.getAppliedOutput() * 12.0);
    SmartDashboard.putNumber("Intake/Roller Current", mIntakeRollerMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Intake/Coral Detected", getCoralDetected());
    SmartDashboard.putNumber("Intake/Pivot Setpoint", mIntakePivotProfiledPID.getGoal().position);
  }
}
