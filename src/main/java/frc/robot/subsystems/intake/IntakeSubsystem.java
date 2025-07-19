package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeConstants.Beambreak;
import frc.robot.subsystems.intake.IntakeConstants.Indexer;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivot;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRoller;
import frc.robot.subsystems.wrist.WristConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax mIntakePivotMotor;
  private final AbsoluteEncoder mEncoder;
  private final TalonFX mIntakeRollerMotor;
  private final SparkFlex mIndexerMotor;
  private boolean mDisableIR;

  // private final DigitalInput mCoralSensor1;
  private final DigitalInput mCoralSensorFront;
  private final DigitalInput mCoralSensorBack;

  private final ProfiledPIDController mIntakePivotProfiledPID;
  private ArmFeedforward mIntakePivotFF;

  private boolean mBackTriggered;
  private boolean mFrontTriggered;
  private Timer mCoralStuckTimer;

  public IntakeSubsystem() {
    mBackTriggered = false;
    this.mDisableIR = false;
    this.mIntakePivotMotor = new SparkMax(IntakePivot.kPivotID, IntakePivot.kMotorType);
    this.mEncoder = mIntakePivotMotor.getAbsoluteEncoder();
    // this.mEncoder.

    this.mIntakePivotProfiledPID = new ProfiledPIDController(
        IntakePivot.kP,
        0,
        IntakePivot.kD,
        new Constraints(IntakePivot.kMaxVelocity, IntakePivot.kMaxAcceleration));
    this.mIntakePivotProfiledPID.setTolerance(IntakeConstants.IntakePivot.kTolerance);
    this.mIntakePivotFF = new ArmFeedforward(IntakePivot.kS, IntakePivot.kG, IntakePivot.kV, IntakePivot.kA);

    this.mIntakeRollerMotor =new TalonFX(IntakeRoller.kRollerID);
    mIntakeRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        // Apply configurations
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeRoller.kRollerCurrentLimit;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeRoller.kRollerCurrentLimit;
    rollerConfig.Voltage.PeakForwardVoltage = 12;
    rollerConfig.Voltage.PeakReverseVoltage = -12;

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted =  IntakeRoller.kInverted 
        ? InvertedValue.CounterClockwise_Positive 
        : InvertedValue.Clockwise_Positive;

    mIntakeRollerMotor.getConfigurator().apply(rollerConfig, 1.0);

    this.mIndexerMotor = new SparkFlex(Indexer.kIndexerID, Indexer.kMotorType);
    mIndexerMotor.configure(
        Indexer.kIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    mIntakeRollerMotor.getConfigurator().apply(rollerConfig, 1.0);

    this.setDefaultCommand(enableFFCmd());

    SmartDashboard.putNumber("Intake/Indexer Volts", 1);
    SmartDashboard.putNumber("Intake/Intake Volts", 1);
    SmartDashboard.putNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);

    SmartDashboard.putNumber("Intake/PivotKp", IntakeConstants.IntakePivot.kP);
    SmartDashboard.putNumber("Intake/PivotKd", IntakeConstants.IntakePivot.kD);
    SmartDashboard.putNumber("Intake/PivotKa", IntakeConstants.IntakePivot.kA);
    SmartDashboard.putNumber("Intake/PivotKv", IntakeConstants.IntakePivot.kV);
    SmartDashboard.putNumber("Intake/PivotKs", IntakeConstants.IntakePivot.kS);

    SmartDashboard.putNumber("Intake/Tunable Setpoint", 0.0);
    SmartDashboard.putNumber("Intake/Tunable Pivot Voltage", 0.0);

    SmartDashboard.putNumber("Intake/Max Velocity", IntakeConstants.IntakePivot.kMaxVelocity);
    SmartDashboard.putNumber(
        "Intake/Max Acceleration", IntakeConstants.IntakePivot.kMaxAcceleration);

    mCoralSensorFront = new DigitalInput(Beambreak.kFrontSensorDIOPort);
    mCoralSensorBack = new DigitalInput(Beambreak.kBackSensorDIOPort);

    mCoralStuckTimer = new Timer();
    mCoralStuckTimer.stop();
    mCoralStuckTimer.reset();
  }

  public FunctionalCommand enableFFCmd() {
    return new FunctionalCommand(
        () -> {
          double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);
          mIntakePivotFF = new ArmFeedforward(
              IntakeConstants.IntakePivot.kS,
              newKg,
              IntakeConstants.IntakePivot.kV,
              IntakeConstants.IntakePivot.kA);
        },
        () -> {
          double calculatedOutput = mIntakePivotFF.calculate(Math.toRadians(getEncoderReading() - 10.0), 0.0);
          setVoltsIntakePivot(calculatedOutput);
        },
        (interrupted) -> setVoltsIntakePivot(0),
        () -> false,
        this);
  }

  public void toggleIRSensor() {
    mDisableIR = !mDisableIR;
  }

  public boolean isPIDAtGoalIntakePivot() {
    return mIntakePivotProfiledPID.atGoal();
  }

  public FunctionalCommand setPIDIntakePivotCmd(IntakePivot.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakeConstants.IntakePivot.kP);
          double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakeConstants.IntakePivot.kD);
          double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);
          mIntakePivotFF = new ArmFeedforward(
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
          double calculatedFF = mIntakePivotFF.calculate(
              Math.toRadians(mIntakePivotProfiledPID.getSetpoint().position - 10.0),
              Math.toRadians(mIntakePivotProfiledPID.getSetpoint().velocity));
          double calculatedPID = mIntakePivotProfiledPID.calculate(encoderReading);
          setVoltsIntakePivot(calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/PID Output", calculatedPID);
          SmartDashboard.putNumber("Intake/FF Output", calculatedFF);
        },
        (interrupted) -> setVoltsIntakePivot(mIntakePivotFF.calculate(Math.toRadians(getEncoderReading()), 0.0)),
        () -> false, // isPIDAtGoalIntakePivot(),
        this);
  }

  public FunctionalCommand setEndablePIDIntakePivotCmd(IntakePivot.Setpoints pSetpoint) {
    return new FunctionalCommand(
        () -> {
          double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakeConstants.IntakePivot.kP);
          double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakeConstants.IntakePivot.kD);
          double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);
          mIntakePivotFF = new ArmFeedforward(
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
          double calculatedFF = mIntakePivotFF.calculate(
              Math.toRadians(mIntakePivotProfiledPID.getSetpoint().position - 10.0),
              Math.toRadians(mIntakePivotProfiledPID.getSetpoint().velocity));
          double calculatedPID = mIntakePivotProfiledPID.calculate(encoderReading);
          setVoltsIntakePivot(calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/Full Output", calculatedPID + calculatedFF);
          SmartDashboard.putNumber("Intake/PID Output", calculatedPID);
          SmartDashboard.putNumber("Intake/FF Output", calculatedFF);
        },
        (interrupted) -> setVoltsIntakePivot(mIntakePivotFF.calculate(Math.toRadians(getEncoderReading()), 0.0)),
        () -> isPIDAtGoalIntakePivot(),
        this);
  }

  public FunctionalCommand setTunablePIDIntakeCommand() {
    return setTunablePIDIntakeCommand(SmartDashboard.getNumber("Intake/Tunable Setpoint", 0.0));
  }

  public FunctionalCommand setTunablePIDIntakeCommand(double pSetpoint) {
    return new FunctionalCommand(
        () -> {
          double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakeConstants.IntakePivot.kP);
          double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakeConstants.IntakePivot.kD);
          double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakeConstants.IntakePivot.kG);
          double newKa = SmartDashboard.getNumber("Intake/PivotKa", IntakeConstants.IntakePivot.kA);
          double newKv = SmartDashboard.getNumber("Intake/PivotKv", IntakeConstants.IntakePivot.kV);
          double newKs = SmartDashboard.getNumber("Intake/PivotKs", IntakeConstants.IntakePivot.kS);
          double newMaxVel = SmartDashboard.getNumber(
              "Intake/Max Velocity", IntakeConstants.IntakePivot.kMaxVelocity);
          double newMaxAccel = SmartDashboard.getNumber(
              "Intake/Max Acceleration", IntakeConstants.IntakePivot.kMaxAcceleration);
          mIntakePivotFF = new ArmFeedforward(newKs, newKg, newKv, newKa);
          mIntakePivotProfiledPID.setConstraints(new Constraints(newMaxVel, newMaxAccel));
          mIntakePivotProfiledPID.setPID(newKp, 0.0, newKd);
          mIntakePivotProfiledPID.reset(getEncoderReading());
          mIntakePivotProfiledPID.setGoal(pSetpoint);
        },
        () -> {
          double encoderReading = getEncoderReading();
          double calculatedFF = mIntakePivotFF.calculate(
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

  public FunctionalCommand autonSetIndexCoralCmd() {
    return new FunctionalCommand(
        () -> {
          mBackTriggered = false;
        },
        () -> {
          if (getCoralDetectedBack() || getCoralDetectedFront()) {
            mBackTriggered = true;
          }
          
          if (mBackTriggered) {
            setVoltsIndexer(IntakeConstants.Indexer.kIntakeVoltsSlow);
          } 
          else {
            setVoltsIndexer(IntakeConstants.Indexer.kIntakeVolts);
          }
        },
        (interrupted) -> {
          setVoltsIndexer(0.0);
        },
        () -> {
          return getCoralDetectedFront();
        });
  }

  public FunctionalCommand setIndexCoralCmd() {
    return new FunctionalCommand(
        () -> {
          mBackTriggered = false;
        },
        () -> {
          if (getCoralDetectedBack() || getCoralDetectedFront()) {
            mBackTriggered = true;
          }
          

          if (mBackTriggered) {
            setVoltsIndexer(IntakeConstants.Indexer.kIntakeVoltsSlow);
          } else {
            setVoltsIndexer(IntakeConstants.Indexer.kIntakeVolts);
          }
        },
        (interrupted) -> {
          setVoltsIndexer(0.0);
          mBackTriggered = false;
        },
        () -> {
          return getCoralDetectedFront();
        });
  }
  /* 
  public RepeatCommand setIndexCoralCmd() {
    return new RepeatCommand(
      new ParallelDeadlineGroup(new WaitCommand(IntakeConstants.Indexer.kIntakeStuckTime), initialIndexCommand())
      .andThen(new ParallelDeadlineGroup(new WaitCommand(IntakeConstants.Indexer.kIntakeReverseTime), reverseIndexCommand()))
    );
  }

  private FunctionalCommand initialIndexCommand() {
    return new FunctionalCommand(
        () -> {
          mBackTriggered = getCoralDetectedBack();
          mFrontTriggered = getCoralDetectedFront();
        },
        () -> {
          if (getCoralDetectedBack()) {
            mBackTriggered = true;
          }
          if(getCoralDetectedFront()) {
            mFrontTriggered = true;
          }
          if (mFrontTriggered = true) setVoltsIndexer(IntakeConstants.Indexer.kIntakeVoltsHold);
          else if (mBackTriggered) setVoltsIndexer(IntakeConstants.Indexer.kIntakeVoltsSlow); 
          else setVoltsIndexer(IntakeConstants.Indexer.kIntakeVolts);
        },
        (interrupted) -> {
          setVoltsIndexer(0.0);
          mBackTriggered = false;
        },
        () -> {
          return getCoralDetectedFront();
        });
  } 

  private FunctionalCommand reverseIndexCommand() {
    return new FunctionalCommand(
      () -> {}, 
      () -> {
        setVoltsIndexer(IntakeConstants.Indexer.kOuttakeVolts);
      }, 
      (interrupted) -> setVoltsIndexer(0.0), 
      () -> getCoralDetectedBack());
  }
      */

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
    return mDisableIR ? false : !mCoralSensorFront.get();
  }

  public boolean getCoralDetectedBack() {
    return mDisableIR ? true : !mCoralSensorBack.get();
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
        () -> {
        },
        () -> setVoltsIntakePivot(
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
    SmartDashboard.putBoolean("Intake/Coral Detected Back", getCoralDetectedBack());
    SmartDashboard.putBoolean("Intake/Run Slower", mBackTriggered);

    SmartDashboard.putNumber("Intake/Pivot Position", getEncoderReading());
    SmartDashboard.putNumber("Intake/Pivot Current", mIntakePivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Pivot Voltage", mIntakePivotMotor.getBusVoltage());
    SmartDashboard.putNumber("Intake/Roller Current", mIntakeRollerMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Intake/Coral Detected", getCoralDetectedFront());
    SmartDashboard.putNumber("Intake/Pivot Setpoint", mIntakePivotProfiledPID.getGoal().position);
    SmartDashboard.putBoolean("Intake/IR Disabled", mDisableIR);
  }
}
