package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.auton.AutonSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.controls.ControlsSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.telemetry.TelemetrySubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Axes Multipler

  // Subsystems
  private final DriveSubsystem mDrive;
  private final ClawSubsystem mClaw;
  private final WristSubsystem mWrist;
  private final VisionSubsystem mVision;
  private final ElevatorSubsystem mElevator;
  private final ControlsSubsystem mControls;
  private final TelemetrySubsystem mTelemetry;
  private final IntakeSubsystem mIntake;
  private final LEDSubsystem mLEDs;
  private final AutonSubsystem mAutons;
  private final ClimbSubsystem mCLimb;

  public RobotContainer() {
    mTelemetry = new TelemetrySubsystem();
    mClaw = new ClawSubsystem();
    mWrist = new WristSubsystem();
    mElevator = new ElevatorSubsystem();
    mIntake = new IntakeSubsystem();
    mCLimb = new ClimbSubsystem();

    switch (Constants.currentMode) {
      case REAL:
        mDrive =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                mTelemetry);
        break;

      case SIM:
        mDrive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                mTelemetry);
        break;

      default:
        mDrive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                mTelemetry);
        break;
    }

    mLEDs = new LEDSubsystem(mClaw, mIntake, mElevator, mWrist, mDrive);

    mVision =
        new VisionSubsystem(mDrive, () -> mDrive.getRotation(), () -> mDrive.getModulePositions());
    mControls = new ControlsSubsystem(mDrive, mVision, mWrist, mElevator, mIntake, mClaw, mCLimb);
    mAutons = new AutonSubsystem(mDrive, mWrist, mVision, mClaw, mElevator, mIntake);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    mControls.initDriverController();
    mControls.initOperatorButtonboard();
    mControls.initDrivebase();
    mControls.initTriggers();
    // mControls.initTuningDrive();
    // mControls.initIntakeTuning();
    // mControls.initElevatorTuning();
    //mControls.initWristTuning();
  }

  public Command getAutonomousCommand() {
    return mAutons.getChosenAuton();
  }

  public DriveSubsystem getDrivetrain() {
    return mDrive;
  }

  public TelemetrySubsystem getTelemetry() {
    return mTelemetry;
  }

  public Command getPathPlannerAuto() {
    return mAutons.getChosenAuton();
  }
}
