package frc.robot.subsystems.wrist;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.controls.StateTracker.CoralLevel;
import frc.robot.util.debugging.LoggedTunableNumber;


public class WristSubsystem extends SubsystemBase{
    @AutoLogOutput(key = "Wrist/Slot")
    public int slot = 0;

    private static final LoggedTunableNumber k0P = new LoggedTunableNumber("Wrist/Slot0/P", WristConstants.k0P);
    private static final LoggedTunableNumber k0I = new LoggedTunableNumber("Wrist/Slot0/I", WristConstants.k0I);
    private static final LoggedTunableNumber k0D = new LoggedTunableNumber("Wrist/Slot0/D", WristConstants.k0D);
    private static final LoggedTunableNumber k0MaxV = new LoggedTunableNumber("Wrist/Slot0/MaxV", WristConstants.k0MaxVelocity);
    private static final LoggedTunableNumber k0MaxA = new LoggedTunableNumber("Wrist/Slot0/MaxA", WristConstants.k0MaxAcceleration);
    private static final LoggedTunableNumber k0S = new LoggedTunableNumber("Wrist/Slot0/S", WristConstants.k0S);
    private static final LoggedTunableNumber k0V = new LoggedTunableNumber("Wrist/Slot0/V", WristConstants.k0V);
    private static final LoggedTunableNumber k0A = new LoggedTunableNumber("Wrist/Slot0/A", WristConstants.k0A);
    private static final LoggedTunableNumber k0G = new LoggedTunableNumber("Wrist/Slot0/G", WristConstants.k0G);

    private static final LoggedTunableNumber k1P = new LoggedTunableNumber("Wrist/Slot1/P", WristConstants.k1P);
    private static final LoggedTunableNumber k1I = new LoggedTunableNumber("Wrist/Slot1/I", WristConstants.k1I);
    private static final LoggedTunableNumber k1D = new LoggedTunableNumber("Wrist/Slot1/D", WristConstants.k1D);
    private static final LoggedTunableNumber k1MaxV = new LoggedTunableNumber("Wrist/Slot1/MaxV", WristConstants.k1MaxVelocity);
    private static final LoggedTunableNumber k1MaxA = new LoggedTunableNumber("Wrist/Slot1/MaxA", WristConstants.k1MaxAcceleration);
    private static final LoggedTunableNumber k1S = new LoggedTunableNumber("Wrist/Slot1/S", WristConstants.k1S);
    private static final LoggedTunableNumber k1V = new LoggedTunableNumber("Wrist/Slot1/V", WristConstants.k1V);
    private static final LoggedTunableNumber k1A = new LoggedTunableNumber("Wrist/Slot1/A", WristConstants.k1A);
    private static final LoggedTunableNumber k1G = new LoggedTunableNumber("Wrist/Slot0/G", WristConstants.k1G);

    private static final LoggedTunableNumber k2P = new LoggedTunableNumber("Wrist/Slot2/P", WristConstants.k2P);
    private static final LoggedTunableNumber k2I = new LoggedTunableNumber("Wrist/Slot2/I", WristConstants.k2I);
    private static final LoggedTunableNumber k2D = new LoggedTunableNumber("Wrist/Slot2/D", WristConstants.k2D);
    private static final LoggedTunableNumber k2MaxV = new LoggedTunableNumber("Wrist/Slot2/MaxV", WristConstants.k2MaxVelocity);
    private static final LoggedTunableNumber k2MaxA = new LoggedTunableNumber("Wrist/Slot2/MaxA", WristConstants.k2MaxAcceleration);
    private static final LoggedTunableNumber k2S = new LoggedTunableNumber("Wrist/Slot2/S", WristConstants.k2S);
    private static final LoggedTunableNumber k2V = new LoggedTunableNumber("Wrist/Slot2/V", WristConstants.k2V);
    private static final LoggedTunableNumber k2A = new LoggedTunableNumber("Wrist/Slot2/A", WristConstants.k2A);
    private static final LoggedTunableNumber k2G = new LoggedTunableNumber("Wrist/Slot2/G", WristConstants.k2G);

    private final WristIO kWristHardware;
    private final WristIOInputsAutoLogged KWristInputs = new WristIOInputsAutoLogged();

    private ProfiledPIDController kWristPID;
    private ArmFeedforward kWristFF;

    public WristSubsystem(WristIO wristIO){
        kWristHardware = wristIO;

        this.kWristPID = new ProfiledPIDController(
            WristConstants.k0P,
            WristConstants.k0I,
            WristConstants.k0D,           
            new Constraints(WristConstants.k0MaxVelocity, WristConstants.k0MaxAcceleration));
        
        this.kWristPID.setTolerance(WristConstants.k0Tolerance);
        this.kWristFF = new ArmFeedforward(
            WristConstants.k0S, 
            WristConstants.k0G, 
            WristConstants.k0V, 
            WristConstants.k0A);

        this.setDefaultCommand(enableFFCmd());
    }

    @Override
    public void periodic(){
        kWristHardware.updateInputs(KWristInputs);

        Logger.processInputs("Wrist", KWristInputs);

        if(DriverStation.isDisabled()){
            kWristHardware.stop();
        }

        Logger.recordOutput("Wrist/atGoal", kWristPID.atGoal());

        switch(slot) {
            case 0:
                LoggedTunableNumber.ifChanged(hashCode(), () -> {
                updatePIDandFF(k0P.get(), k0I.get(), k0D.get(), k0MaxV.get(), k0MaxA.get(), k0S.get(), k0V.get(), k0A.get(), k0G.get());
                }, k0P, k0I, k0D, k0MaxV, k0MaxA, k0S, k0V, k0A, k0G);
                break;
            case 1:
                LoggedTunableNumber.ifChanged(hashCode(), () -> {
                updatePIDandFF(k1P.get(), k1I.get(), k1D.get(), k1MaxV.get(), k1MaxA.get(), k1S.get(), k1V.get(), k1A.get(), k1G.get());
                }, k1P, k1I, k1D, k1MaxV, k1MaxA, k1S, k1V, k1A, k1G);
                break;
            case 2:
                LoggedTunableNumber.ifChanged(hashCode(), () -> {
                updatePIDandFF(k2P.get(), k2I.get(), k2D.get(), k2MaxV.get(), k2MaxA.get(), k2S.get(), k2V.get(), k2A.get(), k2G.get());
                }, k2P, k2I, k2D, k2MaxV, k2MaxA, k2S, k2V, k2A, k2G);
                break;
            default:
                Logger.recordOutput("STOP DUMBAHH", "ELEVATOR");
            }

        Logger.recordOutput("Wrist/Position Setpoint", kWristPID.getSetpoint().position);
        Logger.recordOutput("Wrist/Velocity Setpoint", kWristPID.getSetpoint().velocity);

        Logger.recordOutput("Wrist/At Setpoint", isPIDAtGoal());

        Logger.recordOutput("Wrist/controller/kP", kWristPID.getP());
        Logger.recordOutput("Wrist/controller/kI", kWristPID.getI());
        Logger.recordOutput("Wrist/controller/kD", kWristPID.getD());
        Logger.recordOutput("Wrist/controller/kMaxV", kWristPID.getConstraints().maxVelocity);
        Logger.recordOutput("Wrist/controller/kMaxA", kWristPID.getConstraints().maxAcceleration);
        Logger.recordOutput("Wrist/controller/kS", kWristFF.getKs());
        Logger.recordOutput("Wrist/controller/kV", kWristFF.getKv());
        Logger.recordOutput("Wrist/controller/kA", kWristFF.getKa());
        Logger.recordOutput("Wrist/controller/kG", kWristFF.getKg());

        stopIfLimit();
    }

    public FunctionalCommand enableFFCmd() {

        return new FunctionalCommand(
            () -> {
            System.out.println("Wrist FF Running");
            },
            () -> {
            double calculatedOutput = kWristFF.calculate(Units.degreesToRadians(getPosition()), 0);
            kWristHardware.setVoltage(calculatedOutput);
            },
            (interrupted) -> kWristHardware.setVoltage(kWristFF.calculate(Units.degreesToRadians(getPosition()), 0)),
            () -> false,
            this);
    }

    private boolean isOutOfBounds(double pInput) {
        return (pInput > 0 && getPosition() >= WristConstants.kForwardSoftLimit)
            || (pInput < 0 && getPosition() <= WristConstants.kReverseSoftLimit);
      }

    public FunctionalCommand setVoltsCmd(double pVoltage) {
        return new FunctionalCommand(
            () -> {
            },
            () -> {
            kWristHardware.setVoltage(pVoltage);
            },
            (interrupted) -> kWristHardware.setVoltage(0),
            () -> false,
            this);
    }
    
    public Command setPIDCmd(WristConstants.Setpoints pSetpoint, BooleanSupplier hasGamePiece) {
        return new InstantCommand(
          () -> {
            if(hasGamePiece.getAsBoolean()) {
              setSlot(1);
            } else if(pSetpoint.equals(WristConstants.Setpoints.THROW_ALGAE)) {
              setSlot(2);
            } else {
              setSlot(0);
            }
          }
        )
        .andThen(new FunctionalCommand(
            () -> {
              Logger.recordOutput("Wrist/Setpoint", pSetpoint.getPos());
              kWristPID.reset(getPosition());
              kWristPID.setGoal(pSetpoint.getPos());
            },
            () -> {
              double calculatedFF = kWristFF.calculate(
                  Math.toRadians(kWristPID.getSetpoint().position),
                  Math.toRadians(kWristPID.getSetpoint().velocity));
              double calculatedPID = kWristPID.calculate(getPosition());
              Logger.recordOutput("Wrist/Full Output", calculatedPID + calculatedFF);
              Logger.recordOutput("Wrist/PID Output", calculatedPID);
              Logger.recordOutput("Wrist/FF Output", calculatedFF);
              kWristHardware.setVoltage(calculatedPID + calculatedFF);
            },
            (interrupted) -> kWristHardware.setVoltage(kWristFF.calculate(Math.toRadians(getPosition()), 0.0)),
            () -> isPIDAtGoal(),
            this));
      }
    

    
    public FunctionalCommand setTunablePIDCmd(double pSetpoint) {
        return new FunctionalCommand(
            () -> {
            kWristPID.reset(getPosition());
            kWristPID.setGoal(pSetpoint);
            },
            () -> {
            double encoderReading = getPosition();
            double calculatedPID = kWristPID.calculate(encoderReading);
            double calculatedFF = kWristFF.calculate(
                Units.degreesToRadians(kWristPID.getSetpoint().position),
                Units.degreesToRadians(kWristPID.getSetpoint().velocity));

            kWristHardware.setVoltage(calculatedPID + calculatedFF);
           Logger.recordOutput("Wrist/Full Output", calculatedPID + calculatedFF);
           Logger.recordOutput("Wrist/PID Output", calculatedPID);
           Logger.recordOutput("Wrist/FF Output", calculatedFF);
            },
            (interrupted) -> kWristHardware.setVoltage(0),
            () -> isPIDAtGoal(),
            this);
    }

    public boolean isPIDAtGoal() {
        return kWristPID.atGoal();
    }

    public double getPosition(){
        return KWristInputs.positionMeters;
    }


    public Command coralLevelToPIDCmd(CoralLevel pCoralLevel, BooleanSupplier hasGamePiece) {
        WristConstants.Setpoints elevatorSetpoint = WristConstants.Setpoints.L1;

        if(pCoralLevel == CoralLevel.B3) {
        elevatorSetpoint = WristConstants.Setpoints.L4;
        } else
        if(pCoralLevel == CoralLevel.B2) {
        elevatorSetpoint = WristConstants.Setpoints.L3;
        } else
        if(pCoralLevel == CoralLevel.B1) {
        elevatorSetpoint = WristConstants.Setpoints.L2;
        } 

        return setPIDCmd(elevatorSetpoint, hasGamePiece);
    }


    public void updatePIDandFF(double kP, double kI, double kD, double kMaxV, double kMaxA, double kS, double kV, double kA, double kG) {
        kWristPID.setPID(kP, kI, kD);
        kWristPID.setConstraints(new Constraints(kMaxV, kMaxA));
        kWristFF = new ArmFeedforward(kS, kG, kV, kA);
    }

    public Command setSlotCommand(int slot) {
        return new InstantCommand(() -> {
          setSlot(slot);
        });
    }

    public void setSlot(int slot) {
        this.slot = slot;
        switch(this.slot) {
          case 1:
            updatePIDandFF(k1P.get(), k1I.get(), k1D.get(), k1MaxV.get(), k1MaxA.get(), k1S.get(), k1V.get(), k1A.get(), k1G.get());
            break;
          case 2:
            updatePIDandFF(k2P.get(), k2I.get(), k2D.get(), k2MaxV.get(), k2MaxA.get(), k2S.get(), k2V.get(), k2A.get(), k2G.get());
            break;
          default:
            if(slot != 0) DriverStation.reportWarning("INVALID SLOT PASSED IN, DEFAULTING TO SLOT 0", true);
            updatePIDandFF(k0P.get(), k0I.get(), k0D.get(), k0MaxV.get(), k0MaxA.get(), k0S.get(), k0V.get(), k0A.get(), k0G.get());
        }
    }

    private void stopIfLimit() {
        if (isOutOfBounds(KWristInputs.motorOutput)) {
          kWristHardware.setVoltage(0);
        }
    }
}
