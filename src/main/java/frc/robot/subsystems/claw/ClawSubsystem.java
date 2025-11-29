package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.claw.ClawConstants;
import frc.robot.subsystems.claw.claw.ClawIO;
import frc.robot.subsystems.claw.claw.ClawIOInputsAutoLogged;
import frc.robot.subsystems.claw.claw.ClawConstants.RollerSpeed;
import frc.robot.subsystems.claw.sensor.SensorIO;
import frc.robot.subsystems.claw.sensor.SensorIOInputsAutoLogged;
import frc.robot.subsystems.climb.grabber.GrabberIO;
import frc.robot.subsystems.climb.pulley.PulleyConstants;
import frc.robot.subsystems.climb.pulley.PulleyIO;
import frc.robot.subsystems.controls.StateTracker.GamePiece;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ClawSubsystem extends SubsystemBase{

    private final ClawIO kClawHardware;
    private final SensorIO kSensorHardware;

    private final ClawIOInputsAutoLogged kClawInputs = new ClawIOInputsAutoLogged();
    private final SensorIOInputsAutoLogged kSensorInputs = new SensorIOInputsAutoLogged();
    private final Debouncer kHasPieceDebouncer = new Debouncer(0.01, DebounceType.kRising);

    public ClawSubsystem(ClawIO clawIO, SensorIO sensorIO){
        kClawHardware = clawIO;
        kSensorHardware = sensorIO;
    }

    @Override
    public void periodic(){
        kClawHardware.updateInputs(kClawInputs);
        Logger.processInputs("Claw/Claw", kClawInputs);

        kSensorHardware.updateInputs(kSensorInputs);
        Logger.processInputs("Claw/Sensors", kSensorInputs);

        Logger.recordOutput("Claw/Has Peice", hasPiece());

        if(DriverStation.isDisabled()){
            kClawHardware.stop();
        }
        
    }

    public void setClaw(ClawConstants.RollerSpeed pVoltage) {
        setClaw(pVoltage.get());
    }

    public void setClaw(double pVoltage) {
        kClawHardware.setVoltage(pVoltage);
    }

    public InstantCommand setClawCmdInstant(double pVoltage) {
        return new InstantCommand(()-> setClaw(pVoltage));
    }

    public FunctionalCommand setClawCmd(double pVoltage) {
        return new FunctionalCommand(
            () -> setClaw(pVoltage), () -> setClaw(pVoltage), (interrupted) -> {
            }, () -> false, this);
    }

    public FunctionalCommand holdCoralCmd() {
        return new FunctionalCommand(
            () -> setClaw(ClawConstants.RollerSpeed.HOLD_CORAL),
            () -> setClaw(ClawConstants.RollerSpeed.HOLD_CORAL),
            (interrupted) -> setClaw(0.0),
            () -> false,
            this);
    }

    public boolean hasPiece(){
        return kHasPieceDebouncer.calculate(CANRangeHasPiece());
        // || beambreakHasPiece();
    }

    private boolean beambreakHasPiece() {
        return kSensorHardware.getBeamBreakValue();
    }



    public GamePiece CANRangeGuessGamePiece() {
        if((kSensorInputs.distanceFromClaw < 0.08) && 
        (kSensorInputs.signalStrength > 3000) && 
        (kSensorInputs.signalStrength < 20000) &&
        (kSensorInputs.ambience < 20)) return GamePiece.Algae;
        
        if((kSensorInputs.distanceFromClaw < ClawConstants.coralDetectionCutoff) && 
        (kSensorInputs.signalStrength > 20000)  && 
        (kSensorInputs.ambience < 20)) return GamePiece.Coral;

        return null;
    }

    private boolean CANRangeHasPiece() {
        return (kSensorInputs.distanceFromClaw < 0.06) && 
        (kSensorInputs.signalStrength > 8000) &&
        (kSensorInputs.ambience < 20);
    }

    public FunctionalCommand intakeCoralCmd() {
        return new FunctionalCommand(
            () -> setClaw(
                ClawConstants.RollerSpeed.INTAKE_CORAL),
            () -> {
            setClaw(ClawConstants.RollerSpeed.INTAKE_CORAL);
            },
            (interrupted) -> setClaw(ClawConstants.RollerSpeed.HOLD_CORAL),
            () -> hasPiece(),
            this);
    }

    public FunctionalCommand scoreCoralCmd(ClawConstants.RollerSpeed speed) {
        return new FunctionalCommand(
            () -> setClaw(speed),
            () -> {
            setClaw(speed);
            },
            (interrupted) -> setClaw(speed),
            () -> !hasPiece(),
            this);
    }

    public FunctionalCommand scoreCoralCmd() {
        return new FunctionalCommand(
            () -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
            () -> {
            setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF);
            },
            (interrupted) -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
            () -> !hasPiece(),
            this);
    }

    public FunctionalCommand scoreReverseCoralCmd() {
        return new FunctionalCommand(
            () -> setClaw(ClawConstants.RollerSpeed.OUTTAKE_REEF),
            () -> {
            setClaw(ClawConstants.RollerSpeed.EJECT_CORAL);
            },
            (interrupted) -> setClaw(ClawConstants.RollerSpeed.EJECT_CORAL),
            () -> !hasPiece(),
            this);
    }

    public FunctionalCommand groundAlgae() {
        return new FunctionalCommand(
            () -> setClaw(ClawConstants.RollerSpeed.GROUND_ALGAE),
            () -> {
            setClaw(ClawConstants.RollerSpeed.GROUND_ALGAE);
            },
            (interrupted) -> setClaw(ClawConstants.RollerSpeed.HOLD_ALGAE),
            () -> !hasPiece(),
            this);
    }

    public FunctionalCommand throwAlgae(WristSubsystem mWrist, ElevatorSubsystem mElevator) {
        return new FunctionalCommand(
            () -> setClaw(ClawConstants.RollerSpeed.HOLD_ALGAE),
            () -> {
            if (mWrist.getPosition() >= WristConstants.throwAlgaePos)
                setClaw(ClawConstants.RollerSpeed.SCORE_BARGE);
            },
            (interrupted) -> {
            if (mWrist.getPosition() >= WristConstants.throwAlgaePos)
                setClaw(ClawConstants.RollerSpeed.SCORE_BARGE);
            },
            () -> mWrist.getPosition() >= WristConstants.throwAlgaePos,
            this);
    }

    public FunctionalCommand autonThrowAlgae(WristSubsystem mWrist, ElevatorSubsystem mElevator) {
        return new FunctionalCommand(
            () -> setClaw(ClawConstants.RollerSpeed.HOLD_ALGAE),
            () -> {
            if (mWrist.getPosition() >= WristConstants.throwAlgaePos)
                setClaw(ClawConstants.RollerSpeed.SCORE_BARGE);
            },
            (interrupted) -> {
            if (mWrist.getPosition() >= WristConstants.throwAlgaePos)
                setClaw(ClawConstants.RollerSpeed.SCORE_BARGE);
            },
            () -> (mWrist.getPosition() > WristConstants.throwAlgaePos + 6),
            this);
    }
    
}
