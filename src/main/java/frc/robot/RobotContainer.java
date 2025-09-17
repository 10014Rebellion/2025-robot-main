package frc.robot;

import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.grabber.*;
import frc.robot.subsystems.climb.grabber.GrabberConstants.Grabber.GrabberConfiguration;
import frc.robot.subsystems.climb.grabber.GrabberConstants.Grabber.GrabberHardware;
import frc.robot.subsystems.climb.pulley.*;
import frc.robot.subsystems.climb.grabber.GrabberIOSparkMax;
import frc.robot.subsystems.climb.pulley.PulleyIOSparkMax;
import frc.robot.subsystems.controls.ButtonBindings;
import frc.robot.subsystems.controls.StateTracker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.Drive.DriveState;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDs.LEDSubsystem;
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
import frc.robot.subsystems.auton.AutonSubsystem;


public class RobotContainer {
  // Subsystems
  private final Drive mDrive;
  private final ClawSubsystem mClaw;
  private final WristSubsystem mWrist;
  private final ElevatorSubsystem mElevator;
  private final ButtonBindings mButtonBindings;
  private final TelemetrySubsystem mTelemetry;
  private final IntakeSubsystem mIntake;
  private final LEDSubsystem mLEDs;
  private final AutonSubsystem mAutons;
  private final ClimbSubsystem mClimb;
  private final StateTracker mStateStracker;

  public RobotContainer() {
    mStateStracker = new StateTracker();
    mTelemetry = new TelemetrySubsystem();
    mClaw = new ClawSubsystem();
    mWrist = new WristSubsystem();
    mElevator = new ElevatorSubsystem();
    mIntake = new IntakeSubsystem();
    mClimb = new ClimbSubsystem(
      new GrabberIOSparkMax(
        (GrabberConstants.Grabber.grabberHardware), 
        (GrabberConstants.Grabber.motorConfiguration)),
      new PulleyIOSparkMax(
        (PulleyConstants.Pulley.pulleyHardware), 
        (PulleyConstants.Pulley.motorConfiguration), 
        (PulleyConstants.Pulley.encoderConfiguration)));
    mLEDs = new LEDSubsystem();

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

    mButtonBindings = new ButtonBindings(mDrive, mElevator, mIntake, mWrist, mClaw, mClimb, mLEDs, mStateStracker);

    // mControls = new ControlsSubsystem(mDrive, mVision, mWrist, mElevator, mIntake, mClaw, mCLimb);
    mAutons = new AutonSubsystem(mDrive, mWrist, mClaw, mElevator, mIntake);

    configureButtonBindings();
  }

  // DO NOT INIT TRIGGERS INSIDE OF HERE UNLESS YOU WANNA DO IT IN AUTON AS WELL!!!
  private void configureButtonBindings() {
    // CommandXboxController controller = new CommandXboxController(3);
    // controller.a().onTrue(mElevator.setSlotCommand(0));
    // controller.x().onTrue(mElevator.setSlotCommand(1));
    // controller.y().onTrue(mElevator.setSlotCommand(2));
    mButtonBindings.initDriverJoysticks();
    mButtonBindings.initDriverButtons();
    mButtonBindings.initOperatorButtons();
  }

  // Run this in robot.java
  public void initTriggers() {
    mButtonBindings.initTriggers();
    mDrive.setDriveState(DriveState.TELEOP);
  }

  public Drive getDrivetrain() {
    return mDrive;
  }

  public TelemetrySubsystem getTelemetry() {
    return mTelemetry;
  }

  public Command getPathPlannerAuto() {
    return mAutons.getChosenAuton();
  }

  public Command getAutonomousCommand() {
    return mAutons.getChosenAuton();
  }
}
