package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.auton.AutonSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.controls.ControlsSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.sensors.BeambreakSubsystem;
import frc.robot.subsystems.telemetry.TelemetrySubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private final PivotSubsystem mPivot;
  private final ControlsSubsystem mControls;
  private final BeambreakSubsystem mBeambreak;
  private final TelemetrySubsystem mTelemetry;
  private final IntakeSubsystem mIntake;
  private final LEDSubsystem mLEDs;
  private final AutonSubsystem mAutons;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    mClaw = new ClawSubsystem();
    mWrist = new WristSubsystem();
    mElevator = new ElevatorSubsystem();
    mTelemetry = new TelemetrySubsystem();
    mIntake = new IntakeSubsystem();
    mPivot = new PivotSubsystem();
    mBeambreak = new BeambreakSubsystem();
    mLEDs = new LEDSubsystem();

    switch (Constants.currentMode) {
      case REAL:
        mDrive =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        mDrive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        mDrive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    mVision =
        new VisionSubsystem(mDrive, () -> mDrive.getRotation(), () -> mDrive.getModulePositions());
    mControls = new ControlsSubsystem(mDrive, mVision, mWrist, mElevator, mPivot, mIntake, mClaw);
    mAutons = new AutonSubsystem(mDrive, mWrist, mVision, mClaw, mElevator, mPivot, mIntake);
    mAutons.configureNamedCommands();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(mDrive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(mDrive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        mDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        mDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", mDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", mDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    mControls.initDriverController();
    // mControls.tuningDrive();
    mControls.initOperatorButtonboard();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
