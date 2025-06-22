package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.LEDs.LEDSubsystem;
// import frc.robot.subsystems.auton.AutonSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
// import frc.robot.subsystems.controls.ControlsSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKraken;
import frc.robot.subsystems.drive.ModuleIOSim;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import frc.robot.subsystems.drive.ModuleIOFXFXS;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.telemetry.TelemetrySubsystem;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPV;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.Orientation;
import frc.robot.subsystems.wrist.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Axes Multipler

  // Subsystems
  private final Drive mDrive;
  private final ClawSubsystem mClaw;
  private final WristSubsystem mWrist;
  private final ElevatorSubsystem mElevator;
  // private final ControlsSubsystem mControls;
  private final TelemetrySubsystem mTelemetry;
  private final IntakeSubsystem mIntake;
  // private final AutonSubsystem mAutons;
  private final ClimbSubsystem mClimb;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer() {
    mTelemetry = new TelemetrySubsystem();
    mClaw = new ClawSubsystem();
    mWrist = new WristSubsystem();
    mElevator = new ElevatorSubsystem();
    mIntake = new IntakeSubsystem();
    mClimb = new ClimbSubsystem();

    switch (Constants.currentMode) {
      case REAL:
         mDrive = new Drive( 
              new Module[] {
                    new Module("FL", new ModuleIOFXFXS(kFrontLeftHardware )),
                    new Module("FR", new ModuleIOFXFXS(kFrontRightHardware)),
                    new Module("BL", new ModuleIOFXFXS(kBackLeftHardware  )),
                    new Module("BR", new ModuleIOFXFXS(kBackRightHardware ))
              }, 
              new GyroIOPigeon2(), 
              new Vision(new CameraIO[] {
                    new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, Orientation.BACK), 
                    new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, Orientation.BACK)
                }));
        break;

      case SIM:
        mDrive = new Drive( new Module[] {
          new Module("FL", new ModuleIOSim()),
          new Module("FR", new ModuleIOSim()),
          new Module("BL", new ModuleIOSim()),
          new Module("BR", new ModuleIOSim())
        }, new GyroIO() {}, new Vision(new CameraIO[] {
          new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, Orientation.FRONT), 
          new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, Orientation.FRONT)
        }));
        break;

      default:
        mDrive = new Drive( new Module[] {
          new Module("FL", new ModuleIO() {}),
          new Module("FR", new ModuleIO() {}),
          new Module("BL", new ModuleIO() {}),
          new Module("BR", new ModuleIO() {})
        }, new GyroIO() {}, new Vision(new CameraIO[] {
          new CameraIO() {}, new CameraIO() {}
        }));
        break;
    }

    // mControls = new ControlsSubsystem(mDrive, mVision, mWrist, mElevator, mIntake, mClaw, mCLimb);
    // mAutons = new AutonSubsystem(mDrive, mWrist, mVision, mClaw, mElevator, mIntake);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
      mDrive.acceptJoystickInputs(
            () -> - driverController.getLeftY(),
            () -> - driverController.getLeftX(),
            () -> driverController.getRightX(),
            () -> driverController.getHID().getPOV());

      driverController.y()
        .onTrue(Commands.runOnce(() -> mDrive.resetGyro()));

    // initTeleop();
    // mControls.initTuningDrive();
    // mControls.initIntakeTuning();
    // mControls.initElevatorTuning();
    // mControls.initWristTuning();
  }

  // private void initTeleop() {
  //   mControls.initDriverController();
  //   mControls.initOperatorButtonboard();
  //   mControls.initDrivebase();
  // }

  public void initTriggers() {
    // mControls.initTriggers();
  }

  // public Command getAutonomousCommand() {
  //   return mAutons.getChosenAuton();
  // }

  public Drive getDrivetrain() {
    return mDrive;
  }

  public TelemetrySubsystem getTelemetry() {
    return mTelemetry;
  }

  // public Command getPathPlannerAuto() {
  //   return mAutons.getChosenAuton();
  // }
}
