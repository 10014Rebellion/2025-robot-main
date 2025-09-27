package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.controls.StateTracker.CoralLevel;
import frc.robot.util.debugging.LoggedTunableNumber;

public class ElevatorSubsystem extends SubsystemBase{

  @AutoLogOutput(key = "Elevator/Slot")
  public int slot = 0;

  private static final LoggedTunableNumber k0P = new LoggedTunableNumber("Elevator/Slot0/P", ElevatorConstants.k0P);
  private static final LoggedTunableNumber k0I = new LoggedTunableNumber("Elevator/Slot0/I", ElevatorConstants.k0I);
  private static final LoggedTunableNumber k0D = new LoggedTunableNumber("Elevator/Slot0/D", ElevatorConstants.k0D);
  private static final LoggedTunableNumber k0MaxV = new LoggedTunableNumber("Elevator/Slot0/MaxV", ElevatorConstants.k0MaxVelocity);
  private static final LoggedTunableNumber k0MaxA = new LoggedTunableNumber("Elevator/Slot0/MaxA", ElevatorConstants.k0MaxAcceleration);
  private static final LoggedTunableNumber k0S = new LoggedTunableNumber("Elevator/Slot0/S", ElevatorConstants.k0S);
  private static final LoggedTunableNumber k0V = new LoggedTunableNumber("Elevator/Slot0/V", ElevatorConstants.k0V);
  private static final LoggedTunableNumber k0A = new LoggedTunableNumber("Elevator/Slot0/A", ElevatorConstants.k0A);
  private static final LoggedTunableNumber k0G = new LoggedTunableNumber("Elevator/Slot0/G", ElevatorConstants.k0G);

  private static final LoggedTunableNumber k1P = new LoggedTunableNumber("Elevator/Slot1/P", ElevatorConstants.k1P);
  private static final LoggedTunableNumber k1I = new LoggedTunableNumber("Elevator/Slot1/I", ElevatorConstants.k1I);
  private static final LoggedTunableNumber k1D = new LoggedTunableNumber("Elevator/Slot1/D", ElevatorConstants.k1D);
  private static final LoggedTunableNumber k1MaxV = new LoggedTunableNumber("Elevator/Slot1/MaxV", ElevatorConstants.k1MaxVelocity);
  private static final LoggedTunableNumber k1MaxA = new LoggedTunableNumber("Elevator/Slot1/MaxA", ElevatorConstants.k1MaxAcceleration);
  private static final LoggedTunableNumber k1S = new LoggedTunableNumber("Elevator/Slot1/S", ElevatorConstants.k1S);
  private static final LoggedTunableNumber k1V = new LoggedTunableNumber("Elevator/Slot1/V", ElevatorConstants.k1V);
  private static final LoggedTunableNumber k1A = new LoggedTunableNumber("Elevator/Slot1/A", ElevatorConstants.k1A);
  private static final LoggedTunableNumber k1G = new LoggedTunableNumber("Elevator/Slot0/G", ElevatorConstants.k1G);

  private static final LoggedTunableNumber k2P = new LoggedTunableNumber("Elevator/Slot2/P", ElevatorConstants.k2P);
  private static final LoggedTunableNumber k2I = new LoggedTunableNumber("Elevator/Slot2/I", ElevatorConstants.k2I);
  private static final LoggedTunableNumber k2D = new LoggedTunableNumber("Elevator/Slot2/D", ElevatorConstants.k2D);
  private static final LoggedTunableNumber k2MaxV = new LoggedTunableNumber("Elevator/Slot2/MaxV", ElevatorConstants.k2MaxVelocity);
  private static final LoggedTunableNumber k2MaxA = new LoggedTunableNumber("Elevator/Slot2/MaxA", ElevatorConstants.k2MaxAcceleration);
  private static final LoggedTunableNumber k2S = new LoggedTunableNumber("Elevator/Slot2/S", ElevatorConstants.k2S);
  private static final LoggedTunableNumber k2V = new LoggedTunableNumber("Elevator/Slot2/V", ElevatorConstants.k2V);
  private static final LoggedTunableNumber k2A = new LoggedTunableNumber("Elevator/Slot2/A", ElevatorConstants.k2A);
  private static final LoggedTunableNumber k2G = new LoggedTunableNumber("Elevator/Slot2/G", ElevatorConstants.k2G);

    private final ElevatorIO kElevatorHardware;
    private final ElevatorIOInputsAutoLogged kElevatorInputs = new ElevatorIOInputsAutoLogged();

    private ProfiledPIDController kElevatorPID;
    private ElevatorFeedforward kElevatorFF;


    public ElevatorSubsystem(ElevatorIO elevatorIO){
        kElevatorHardware = elevatorIO;

        this.kElevatorPID = new ProfiledPIDController(
            ElevatorConstants.k0P,
            ElevatorConstants.k0I,
            ElevatorConstants.k0D,           
            new Constraints(ElevatorConstants.k0MaxVelocity, ElevatorConstants.k0MaxAcceleration));
        
        this.kElevatorPID.setTolerance(ElevatorConstants.k0Tolerance);
        this.kElevatorFF = new ElevatorFeedforward(
            ElevatorConstants.k0S, 
            ElevatorConstants.k0G, 
            ElevatorConstants.k0V, 
            ElevatorConstants.k0A);

        this.setDefaultCommand(enableFFCmd());
    }

    @Override
    public void periodic(){
        kElevatorHardware.updateInputs(kElevatorInputs);

        Logger.processInputs("Elevator", kElevatorInputs);

        if(DriverStation.isDisabled()){
            kElevatorHardware.stop();
        }

        Logger.recordOutput("Elevator/atGoal", atGoal());

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

        Logger.recordOutput("Elevator/controller/kP", kElevatorPID.getP());
        Logger.recordOutput("Elevator/controller/kI", kElevatorPID.getI());
        Logger.recordOutput("Elevator/controller/kD", kElevatorPID.getD());
        Logger.recordOutput("Elevator/controller/kMaxV", kElevatorPID.getConstraints().maxVelocity);
        Logger.recordOutput("Elevator/controller/kMaxA", kElevatorPID.getConstraints().maxAcceleration);
        Logger.recordOutput("Elevator/controller/kS", kElevatorFF.getKs());
        Logger.recordOutput("Elevator/controller/kV", kElevatorFF.getKv());
        Logger.recordOutput("Elevator/controller/kA", kElevatorFF.getKa());
        Logger.recordOutput("Elevator/controller/kG", kElevatorFF.getKg());

        stopIfLimit();
    }

    private void stopIfLimit() {
        if (isOutOfBounds(kElevatorInputs.motorOutput)) {
          kElevatorHardware.setVoltage(0);
        }
    }

    private boolean isOutOfBounds(double pInput) {
        return (pInput > 0 && kElevatorInputs.positionMeters >= ElevatorConstants.kForwardSoftLimit)
            || (pInput < 0 && kElevatorInputs.positionMeters <= ElevatorConstants.kReverseSoftLimit);
    }

    public FunctionalCommand setVoltsCmd(double pVoltage){
        return new FunctionalCommand(
            () -> {
            },
            () -> {
                kElevatorHardware.setVoltage(pVoltage);
            },
            (interrupted) -> kElevatorHardware.setVoltage(0),
            () -> false,
            this);
    }

    public FunctionalCommand enableFFCmd() {
        return new FunctionalCommand(
            () -> {
            },
            () -> {
              double calculatedOutput = kElevatorFF.calculate(0);
              kElevatorHardware.setVoltage(calculatedOutput);
            },
            (interrupted) -> kElevatorHardware.setVoltage(kElevatorFF.calculate(0)),
            () -> false,
            this);
    }

    public FunctionalCommand setPIDCmd(ElevatorConstants.Setpoints pSetpoint) {
        return new FunctionalCommand(
            () -> {
            kElevatorPID.reset(kElevatorInputs.positionMeters);
            kElevatorPID.setGoal(pSetpoint.getPos());
            SmartDashboard.putNumber("Elevator/Setpoint", pSetpoint.getPos());
            },
            () -> {
            double encoderReading = kElevatorInputs.positionMeters;
            double calculatedPID = kElevatorFF.calculate(kElevatorPID.getSetpoint().velocity);
            double calculatedFF = kElevatorPID.calculate(encoderReading);
            kElevatorHardware.setVoltage(calculatedPID + calculatedFF);
            },
            (interrupted) -> kElevatorHardware.setVoltage(kElevatorFF.calculate(0.0)),
            () -> atGoal(),
            this);
    }

    public FunctionalCommand setTunablePIDCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> {
              double encoderReading = kElevatorInputs.positionMeters;
              double calculatedPID = kElevatorFF.calculate(kElevatorPID.getSetpoint().velocity);
              double calculatedFF = kElevatorPID.calculate(encoderReading);
    
              kElevatorHardware.setVoltage(calculatedPID + calculatedFF);
              SmartDashboard.putNumber("Elevator/Full Output", calculatedPID + calculatedFF);
              SmartDashboard.putNumber("Elevator/PID Output", calculatedPID);
              SmartDashboard.putNumber("Elevator/FF Output", calculatedFF);
            },
            (interrupted) -> kElevatorHardware.setVoltage(0),
            () -> false,
            this);
    }

    public FunctionalCommand coralLevelToPIDCmd(CoralLevel pCoralLevel) {
        ElevatorConstants.Setpoints elevatorSetpoint = ElevatorConstants.Setpoints.L1;

        if(pCoralLevel == CoralLevel.B3) {
            elevatorSetpoint = ElevatorConstants.Setpoints.L4;
        } 
        
        else if(pCoralLevel == CoralLevel.B2) {
            elevatorSetpoint = ElevatorConstants.Setpoints.L3;
        } 
        
        else if(pCoralLevel == CoralLevel.B1) {
            elevatorSetpoint = ElevatorConstants.Setpoints.L2;
        } 

        return setPIDCmd(elevatorSetpoint);
    }
    
    public boolean atGoal(){
        return kElevatorPID.atGoal();
    }

    public void updatePIDandFF(double kP, double kI, double kD, double kMaxV, double kMaxA, double kS, double kV, double kA, double kG) {
        kElevatorPID.setPID(kP, kI, kD);
        kElevatorPID.setConstraints(new Constraints(kMaxV, kMaxA));
        kElevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }
    
}
