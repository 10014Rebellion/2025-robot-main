package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.grabber.GrabberConstants;
import frc.robot.subsystems.climb.grabber.GrabberIO;
import frc.robot.subsystems.climb.grabber.GrabberIOInputsAutoLogged;
import frc.robot.subsystems.climb.pulley.PulleyConstants;
import frc.robot.subsystems.climb.pulley.PulleyConstants.Pulley;
import frc.robot.subsystems.climb.pulley.PulleyIO;
import frc.robot.subsystems.climb.pulley.PulleyIOInputsAutoLogged;

public class ClimbSubsystem extends SubsystemBase{

    private final GrabberIO kGrabberHardware;
    private final PulleyIO kPulleyHardware;

    private final GrabberIOInputsAutoLogged kGrabberInputs = new GrabberIOInputsAutoLogged();
    private final PulleyIOInputsAutoLogged kPulleyInputs = new PulleyIOInputsAutoLogged();
    

    public ClimbSubsystem(GrabberIO grabberIO, PulleyIO pulleyIO){
        kGrabberHardware = grabberIO;
        kPulleyHardware = pulleyIO;
    }

    @Override
    public void periodic(){

      
      SmartDashboard.putBoolean("end climb?", getEncReadingDeg() <= PulleyConstants.Pulley.Setpoints.EXTENDED_PRE.getPos());
        kGrabberHardware.updateInputs(kGrabberInputs);
        Logger.processInputs("Climb/Grabber", kGrabberInputs);

        kPulleyHardware.updateInputs(kPulleyInputs);
        Logger.processInputs("Climb/Pulley", kPulleyInputs);

        Logger.recordOutput("Climb", getEncReadingDeg() < PulleyConstants.Pulley.Setpoints.EXTENDED.getPos());

        if(DriverStation.isDisabled()){
            kGrabberHardware.stop();
            kPulleyHardware.stop();
        }
        
    }

  public FunctionalCommand setGrabberVoltsCmd(double pVolts) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          kGrabberHardware.setVoltage(pVolts);
        },
        (interrupted) -> kGrabberHardware.setVoltage(0),
        () -> false);
  }

  public FunctionalCommand setPulleyVoltsCmd(double pVolts) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          kPulleyHardware.setVoltage(pVolts);
        },
        (interrupted) -> kPulleyHardware.setVoltage(0),
        () -> false);
  }


  public FunctionalCommand setGrabberVoltsCmd(GrabberConstants.Grabber.VoltageSetpoints pVolts) {
    return setGrabberVoltsCmd(pVolts.getVolts());
  }

  public FunctionalCommand setPulleyVoltsCmd(PulleyConstants.Pulley.VoltageSetpoints pVolts) {
    return setPulleyVoltsCmd(pVolts.getVolts());
  }

  public void setGrabberVolts(double pVolts){
    kGrabberHardware.setVoltage(pVolts);
  }

  @AutoLogOutput(key="Climb/Position USE")
  public double getEncReadingDeg(){
    return kPulleyInputs.posistionDegrees;
  }

  public boolean getBeamBroken(){
    return kGrabberInputs.hasChain;
  }

public boolean isInTolerance(Pulley.Setpoints pSetpoint) {
    return (Math.abs(getEncReadingDeg() - pSetpoint.getPos()) < PulleyConstants.Pulley.kTolerance);
  }

  public FunctionalCommand deployClimb() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        if (getEncReadingDeg() < PulleyConstants.Pulley.Setpoints.STARTROLLING.getPos())
          kGrabberHardware.setVoltage(GrabberConstants.Grabber.VoltageSetpoints.PULL_IN.getVolts());

        if (getEncReadingDeg() > PulleyConstants.Pulley.Setpoints.SLOW_DOWN.getPos())
          kPulleyHardware.setVoltage(PulleyConstants.Pulley.VoltageSetpoints.GO.getVolts());

        if (getEncReadingDeg() <= PulleyConstants.Pulley.Setpoints.SLOW_DOWN.getPos())
          kPulleyHardware.setVoltage(PulleyConstants.Pulley.VoltageSetpoints.GO_SLOW.getVolts());

        if(getEncReadingDeg() <= PulleyConstants.Pulley.Setpoints.EXTENDED_PRE.getPos())
          kPulleyHardware.setVoltage(PulleyConstants.Pulley.VoltageSetpoints.STOP.getVolts());


        
      }, (interrupted) -> {
        kPulleyHardware.setVoltage(PulleyConstants.Pulley.VoltageSetpoints.STOP.getVolts());
        kGrabberHardware.setVoltage(GrabberConstants.Grabber.VoltageSetpoints.PULL_IN.getVolts());
      }, 
        () -> getEncReadingDeg() <= PulleyConstants.Pulley.Setpoints.EXTENDED_PRE.getPos(), this);
  }

  public FunctionalCommand pullClimb() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        if (getEncReadingDeg() < PulleyConstants.Pulley.Setpoints.CLIMBED.getPos())
          kPulleyHardware.setVoltage(PulleyConstants.Pulley.VoltageSetpoints.GO_FRICKEN_FAAAST.getVolts());
        
      }, (interrupted) -> kPulleyHardware.setVoltage(PulleyConstants.Pulley.VoltageSetpoints.STOP.getVolts()), 
        () -> isInTolerance(PulleyConstants.Pulley.Setpoints.CLIMBED), this);
  }
    
    
}