package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  public enum ElevatorState {STOPPED, MOVING};
  private ElevatorState mState;
  private final ElevatorIO mIo;
  private final ElevatorIOInputsAutoLogged mInputs = new ElevatorIOInputsAutoLogged();
  private final PIDController mElevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  public ElevatorSubsystem(ElevatorIO pIo) {
    this.mIo = pIo;
    mElevatorController.setTolerance(ElevatorConstants.kControllerTolerance);
    setDefaultCommand(
      runOnce(
        () -> {
          mIo.disable();
        })
        .andThen(run(() -> {}))
        .withName("Idle")
    );
    mState = ElevatorState.STOPPED;
  }

  @Override
  public void periodic() {
    mIo.updateInputs(mInputs);
    Logger.processInputs("Elevator", mInputs);
  }

  public void stop() {
    mIo.setPercentOutput(0);
    mState = ElevatorState.STOPPED;
  }

  public void doPID(double pSetpoint) {
    double calculatedOutput = mElevatorController.calculate(mIo.getExtension(), pSetpoint);
    mIo.setVoltage(calculatedOutput);
    mState = ElevatorState.MOVING;
  }

  public boolean PIDFinished() {
    return mElevatorController.atSetpoint();
  }
}
