package frc.robot;

public class StateDaddy {
  private static StateDaddy mStateDaddyInstance = null;

  // public static StateEnums.Drive.Current currentDrive;
  // public static StateEnums.Drive.Setpoint setpointDrive;

  // public static StateEnums.Claw.Current currentClaw;
  // public static StateEnums.Claw.Setpoint setpointClaw;

  // public static StateEnums.Intake.Current currentIntake;
  // public static StateEnums.Intake.Setpoint setpointIntake;

  // public static StateEnums.Elevator.Current currentElevator;
  // public static StateEnums.Elevator.Setpoint setpointElevator;

  private StateDaddy() {
    // currentDrive = StateEnums.Drive.Current.MANUAL;
    // setpointDrive = StateEnums.Drive.Setpoint.NONE;

    // currentClaw = StateEnums.Claw.Current.INDEX;
    // setpointClaw = StateEnums.Claw.Setpoint.INDEX;

    // currentIntake = StateEnums.Intake.Current.RETRACTED;
    // setpointIntake = StateEnums.Intake.Setpoint.RETRACTED;

    // currentElevator = StateEnums.Elevator.Current.RETRACTED;
    // setpointElevator = StateEnums.Elevator.Setpoint.RETRACTED;
  }

  public static synchronized StateDaddy getInstance() {
    if (mStateDaddyInstance == null) {
      mStateDaddyInstance = new StateDaddy();
    }
    return mStateDaddyInstance;
  }

  // public static class Intake {
  //   public boolean atSetpoint() {
  //     return
  //       (currentIntake == StateEnums.Intake.Current.MANUAL && setpointIntake ==
  // StateEnums.Intake.Setpoint.NONE) ||
  //       (currentIntake == StateEnums.Intake.Current.DEPLOYED && setpointIntake ==
  // StateEnums.Intake.Setpoint.DEPLOYED) ||
  //       (currentIntake == StateEnums.Intake.Current.RETRACTED && setpointIntake ==
  // StateEnums.Intake.Setpoint.RETRACTED);
  //   }
  // }

  // public static class Drive {
  //   public boolean atSetpoint() {
  //     return
  //       (currentDrive == StateEnums.Drive.Current.MANUAL && setpointDrive ==
  // StateEnums.Drive.Setpoint.NONE) ||
  //       (currentDrive == StateEnums.Drive.Current.AT_SETPOINT &&
  //         (setpointDrive == StateEnums.Drive.Setpoint.BARGE_POSE || setpointDrive ==
  // StateEnums.Drive.Setpoint.REEF_POSE));
  //   }
  // }

  // public static class Elevator {
  //   public boolean atSetpoint() {
  //     return
  //       (currentElevator == StateEnums.Elevator.Current.MANUAL && setpointElevator ==
  // StateEnums.Elevator.Setpoint.NONE) ||
  //       (currentElevator == StateEnums.Elevator.Current.RETRACTED && setpointElevator ==
  // StateEnums.Elevator.Setpoint.RETRACTED) ||
  //       (currentElevator == StateEnums.Elevator.Current.BARGE && setpointElevator ==
  // StateEnums.Elevator.Setpoint.BARGE) ||
  //       (currentElevator == StateEnums.Elevator.Current.L1 && setpointElevator ==
  // StateEnums.Elevator.Setpoint.L1) ||
  //       (currentElevator == StateEnums.Elevator.Current.L2 && setpointElevator ==
  // StateEnums.Elevator.Setpoint.L2) ||
  //       (currentElevator == StateEnums.Elevator.Current.L3 && setpointElevator ==
  // StateEnums.Elevator.Setpoint.L3) ||
  //       (currentElevator == StateEnums.Elevator.Current.L4 && setpointElevator ==
  // StateEnums.Elevator.Setpoint.L4);
  //   }
  // }

}
