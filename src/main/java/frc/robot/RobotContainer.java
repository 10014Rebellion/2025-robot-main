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
  private final DriveSubsystem drive;
  private final ClawSubsystem claw;
  private final WristSubsystem wrist;
  private final VisionSubsystem vision;
  private final ElevatorSubsystem elevator;
  private final PivotSubsystem pivot;
  private final ControlsSubsystem controls;
  private final BeambreakSubsystem beambreak;
  private final TelemetrySubsystem telemetry;
  private final IntakeSubsystem intake;
  private final LEDSubsystem LEDs;
  private final AutonSubsystem autons;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    claw = new ClawSubsystem();
    wrist = new WristSubsystem();
    elevator = new ElevatorSubsystem();
    telemetry = new TelemetrySubsystem();
    intake = new IntakeSubsystem();
    pivot = new PivotSubsystem();
    beambreak = new BeambreakSubsystem();
    LEDs = new LEDSubsystem();

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        drive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        drive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    vision = new VisionSubsystem(drive, () -> drive.getRotation(), () -> drive.getModulePositions());
    controls = new ControlsSubsystem(drive, vision, elevator, pivot, intake, claw);
    autons = new AutonSubsystem(drive, wrist, vision, claw, elevator, pivot, intake);
    autons.configureNamedCommands();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    controls.initDriverController();
    controls.initOperatorButtonboard();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
