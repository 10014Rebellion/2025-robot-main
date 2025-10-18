package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.BeamBreak.BeamBreakIO;
import frc.robot.subsystems.intake.BeamBreak.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.intake.Indexer.IndexerConstants;
import frc.robot.subsystems.intake.Indexer.IndexerIO;
import frc.robot.subsystems.intake.Indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotConstants;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeRoller.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRoller.IntakeRollerIOInputsAutoLogged;


public class IntakeSubsystem extends SubsystemBase{

    private final IndexerIO kIndexerHardware;
    private final IntakePivotIO kIntakePivotHardware;
    private final IntakeRollerIO kIntakeRollerHardware;
    private final BeamBreakIO kFrontSensorHardware;
    private final BeamBreakIO kBackSensorHardware;

    private final IndexerIOInputsAutoLogged kIndexerInputs = new IndexerIOInputsAutoLogged();
    private final IntakePivotIOInputsAutoLogged kIntakePivotInputs = new IntakePivotIOInputsAutoLogged();
    private final IntakeRollerIOInputsAutoLogged kIntakeRollerInputs = new IntakeRollerIOInputsAutoLogged();
    private final BeamBreakIOInputsAutoLogged kFrontSensorInputs = new BeamBreakIOInputsAutoLogged();
    private final BeamBreakIOInputsAutoLogged kBackSensorInputs = new BeamBreakIOInputsAutoLogged();

    private final ProfiledPIDController mIntakePivotProfiledPID;
    private ArmFeedforward mIntakePivotFF;

    private boolean mBackTriggered;
    private Timer mCoralStuckTimer;

    private boolean mDisableIR;

    public IntakeSubsystem(
        IndexerIO indexerIO, 
        IntakePivotIO intakePivotIO, 
        IntakeRollerIO intakeRollerIO, 
        BeamBreakIO frontBeamBreakIO, 
        BeamBreakIO backBeamBreakIO){

        kIndexerHardware = indexerIO;
        kIntakePivotHardware = intakePivotIO;
        kIntakeRollerHardware = intakeRollerIO;
        kFrontSensorHardware = frontBeamBreakIO;
        kBackSensorHardware = backBeamBreakIO;

        mBackTriggered = false;
        this.mDisableIR = false;

        this.mIntakePivotProfiledPID = new ProfiledPIDController(
            IntakePivotConstants.kP,
            0,
            IntakePivotConstants.kD,
            new Constraints(IntakePivotConstants.kMaxVelocity, IntakePivotConstants.kMaxAcceleration));
        this.mIntakePivotProfiledPID.setTolerance(IntakePivotConstants.kTolerance);
        this.mIntakePivotFF = new ArmFeedforward(IntakePivotConstants.kS, IntakePivotConstants.kG, IntakePivotConstants.kV, IntakePivotConstants.kA);

        this.setDefaultCommand(enableFFCmd());

        mCoralStuckTimer = new Timer();
        mCoralStuckTimer.stop();
        mCoralStuckTimer.reset();
    
    }

    @Override
    public void periodic() {
      kIntakePivotHardware.stopIfLimitIntakePivot();
    
        kIndexerHardware.updateInputs(kIndexerInputs);
        Logger.processInputs("Intake/Indexer", kIndexerInputs);

        kIntakePivotHardware.updateInputs(kIntakePivotInputs);
        Logger.processInputs("Intake/IntakePivot", kIntakePivotInputs);

        kIntakeRollerHardware.updateInputs(kIntakeRollerInputs);
        Logger.processInputs("Intake/IntakeRoller", kIntakeRollerInputs);

        kFrontSensorHardware.updateInputs(kFrontSensorInputs);
        Logger.processInputs("Intake/BeamBreak/Front", kFrontSensorInputs);

        kBackSensorHardware.updateInputs(kBackSensorInputs);
        Logger.processInputs("Intake/BeamBreak/Back", kBackSensorInputs);

        if(DriverStation.isDisabled()){
            kIndexerHardware.stop();
            kIntakePivotHardware.stop();
            kIntakeRollerHardware.stop();
        }

    }

    public FunctionalCommand enableFFCmd() {
        return new FunctionalCommand(
            () -> {
              double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakePivotConstants.kG);
              mIntakePivotFF = new ArmFeedforward(
                  IntakePivotConstants.kS,
                  newKg,
                  IntakePivotConstants.kV,
                  IntakePivotConstants.kA);
            },

            () -> {
              double calculatedOutput = mIntakePivotFF.calculate(Math.toRadians(getPivotPosition() - 10.0), 0.0);
              kIntakePivotHardware.setVoltage(calculatedOutput);
            },

            (interrupted) -> kIntakePivotHardware.setVoltage(0),
            () -> false,
            this);
    }

    public void toggleIRSensor() {
        mDisableIR = !mDisableIR;
    }


    @AutoLogOutput(key="Intake/IntakePivot/atGoal")
    public boolean isPIDAtGoalIntakePivot() {
        return mIntakePivotProfiledPID.atGoal();
    }

    public double getPivotPosition(){
        return kIntakePivotInputs.position;
    }

    public FunctionalCommand setPIDIntakePivotCmd(IntakePivotConstants.Setpoints pSetpoint) {
        return new FunctionalCommand(
            () -> {
              double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakePivotConstants.kP);
              double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakePivotConstants.kD);
              double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakePivotConstants.kG);
              mIntakePivotFF = new ArmFeedforward(
                  IntakePivotConstants.kS,
                  newKg,
                  IntakePivotConstants.kV,
                  IntakePivotConstants.kA);
              mIntakePivotProfiledPID.setPID(newKp, 0.0, newKd);
              mIntakePivotProfiledPID.reset(getPivotPosition());
              mIntakePivotProfiledPID.setGoal(pSetpoint.getPos());
            },
            () -> {
              double encoderReading = getPivotPosition();
              double calculatedFF = mIntakePivotFF.calculate(
                  Math.toRadians(mIntakePivotProfiledPID.getSetpoint().position - 10.0),
                  Math.toRadians(mIntakePivotProfiledPID.getSetpoint().velocity));
              double calculatedPID = mIntakePivotProfiledPID.calculate(encoderReading);
              kIntakePivotHardware.setVoltage(calculatedPID + calculatedFF);
              SmartDashboard.putNumber("Intake/Full Output", calculatedPID + calculatedFF);
              SmartDashboard.putNumber("Intake/PID Output", calculatedPID);
              SmartDashboard.putNumber("Intake/FF Output", calculatedFF);
            },
            (interrupted) -> kIntakePivotHardware.setVoltage(mIntakePivotFF.calculate(Math.toRadians(getPivotPosition()), 0.0)),
            () -> false, // isPIDAtGoalIntakePivot(),
            this);
    }

    public FunctionalCommand setEndablePIDIntakePivotCmd(IntakePivotConstants.Setpoints pSetpoint) {
        return new FunctionalCommand(
            () -> {
            double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakePivotConstants.kP);
            double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakePivotConstants.kD);
            double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakePivotConstants.kG);
            mIntakePivotFF = new ArmFeedforward(
                IntakePivotConstants.kS,
                newKg,
                IntakePivotConstants.kV,
                IntakePivotConstants.kA);
            mIntakePivotProfiledPID.setPID(newKp, 0.0, newKd);
            mIntakePivotProfiledPID.reset(getPivotPosition());
            mIntakePivotProfiledPID.setGoal(pSetpoint.getPos());
            },
            () -> {
            double encoderReading = getPivotPosition();
            double calculatedFF = mIntakePivotFF.calculate(
                Math.toRadians(mIntakePivotProfiledPID.getSetpoint().position - 10.0),
                Math.toRadians(mIntakePivotProfiledPID.getSetpoint().velocity));
            double calculatedPID = mIntakePivotProfiledPID.calculate(encoderReading);
            kIntakePivotHardware.setVoltage(calculatedPID + calculatedFF);
            SmartDashboard.putNumber("Intake/Full Output", calculatedPID + calculatedFF);
            SmartDashboard.putNumber("Intake/PID Output", calculatedPID);
            SmartDashboard.putNumber("Intake/FF Output", calculatedFF);
            },
            (interrupted) -> kIntakePivotHardware.setVoltage(mIntakePivotFF.calculate(Math.toRadians(getPivotPosition()), 0.0)),
            () -> isPIDAtGoalIntakePivot(),
            this);
    }

    public FunctionalCommand setTunablePIDIntakeCommand() {
        return setTunablePIDIntakeCommand(SmartDashboard.getNumber("Intake/Tunable Setpoint", 0.0));
    }

    public FunctionalCommand setTunablePIDIntakeCommand(double pSetpoint) {
        return new FunctionalCommand(
            () -> {
            double newKp = SmartDashboard.getNumber("Intake/PivotKp", IntakePivotConstants.kP);
            double newKd = SmartDashboard.getNumber("Intake/PivotKd", IntakePivotConstants.kD);
            double newKg = SmartDashboard.getNumber("Intake/PivotKg", IntakePivotConstants.kG);
            double newKa = SmartDashboard.getNumber("Intake/PivotKa", IntakePivotConstants.kA);
            double newKv = SmartDashboard.getNumber("Intake/PivotKv", IntakePivotConstants.kV);
            double newKs = SmartDashboard.getNumber("Intake/PivotKs", IntakePivotConstants.kS);
            double newMaxVel = SmartDashboard.getNumber(
                "Intake/Max Velocity", IntakePivotConstants.kMaxVelocity);
            double newMaxAccel = SmartDashboard.getNumber(
                "Intake/Max Acceleration", IntakePivotConstants.kMaxAcceleration);
            mIntakePivotFF = new ArmFeedforward(newKs, newKg, newKv, newKa);
            mIntakePivotProfiledPID.setConstraints(new Constraints(newMaxVel, newMaxAccel));
            mIntakePivotProfiledPID.setPID(newKp, 0.0, newKd);
            mIntakePivotProfiledPID.reset(getPivotPosition());
            mIntakePivotProfiledPID.setGoal(pSetpoint);
            },
            () -> {
            double encoderReading = getPivotPosition();
            double calculatedFF = mIntakePivotFF.calculate(
                Math.toRadians(mIntakePivotProfiledPID.getSetpoint().position),
                Math.toRadians(mIntakePivotProfiledPID.getSetpoint().velocity));
            double calculatedPID = mIntakePivotProfiledPID.calculate(encoderReading);
            kIntakePivotHardware.setVoltage(calculatedPID + calculatedFF);
            SmartDashboard.putNumber("Intake/Full Output", calculatedPID + calculatedFF);
            SmartDashboard.putNumber("Intake/PID Output", calculatedPID);
            SmartDashboard.putNumber("Intake/FF Output", calculatedFF);
            },
            (interrupted) -> kIntakePivotHardware.setVoltage(0),
            () -> false, // isPIDAtGoalIntakePivot(),
            this);
    }

    public InstantCommand setVoltsIntakePivotCmd(double pVoltage) {
        return new InstantCommand(
            () -> {
                kIntakePivotHardware.setVoltage(pVoltage);
            });
    }

    public void setVoltsIntakeRoller(double pVoltage) {
        kIntakeRollerHardware.setVoltage(MathUtil.clamp(pVoltage, -12, 12));
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
                kIndexerHardware.setVoltage(IndexerConstants.kIntakeVoltsSlow);
            } 
            else {
                kIndexerHardware.setVoltage(IndexerConstants.kIntakeVolts);
            }
            },
            (interrupted) -> {
                kIndexerHardware.setVoltage(0.0);
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
                kIndexerHardware.setVoltage(IndexerConstants.kIntakeVoltsSlow);
            } else {
                kIndexerHardware.setVoltage(IndexerConstants.kIntakeVolts);
            }
            },
            (interrupted) -> {
                kIndexerHardware.setVoltage(0.0);
            mBackTriggered = false;
            },
            () -> {
            return getCoralDetectedFront();
            });
    }


    public boolean getCoralDetectedFront() {
        return kFrontSensorInputs.isDetected;
    }

    public boolean getCoralDetectedBack() {
        return kBackSensorInputs.isDetected;
    }


    public FunctionalCommand setPivotCmd(double pVoltage) {
        return new FunctionalCommand(
            () -> kIntakePivotHardware.setVoltage(pVoltage),
            () -> kIntakePivotHardware.setVoltage(pVoltage),
            (interrupted) -> kIntakePivotHardware.setVoltage(0.0),
            () -> false);
    }

    public FunctionalCommand setTunablePivotCmd() {
        return new FunctionalCommand(
            () -> {
            },
            () -> kIntakePivotHardware.setVoltage(
                SmartDashboard.getNumber("Intake/Tunable Pivot Voltage", 0.0)
                    + IntakePivotConstants.kS),
            (interrupted) -> kIntakePivotHardware.setVoltage(0.0),
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
        return new InstantCommand(() -> kIndexerHardware.setVoltage(pVoltage));
    }

    }
