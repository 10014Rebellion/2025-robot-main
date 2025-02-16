package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawFFCommand;
import frc.robot.subsystems.claw.ClawPIDCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorFFCommand;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.elevatorPivot.ElevatorPivot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.potentiometer.Potentiometer;
import frc.robot.subsystems.telemetry.Telemetry;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Axes Multipler
  private final double kDriveSwerveMultipler = 0.5;
  private final double kRotationSwerveMultipler = 0.6;

  // Subsystems
  private final Drive drive;
  private final Claw claw;
  private final Elevator elevator;
  private final Potentiometer potentiometer;
  private final Telemetry telemetry;
  private final Intake intake;
  private final ElevatorPivot elevatorPivot;

  // Controller
  private final CommandXboxController pilot = new CommandXboxController(0);
  private final CommandXboxController copilot = new CommandXboxController(1);

  // Field Oriented
  private boolean mSwerveFieldOriented = true;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    claw = new Claw();
    elevator = new Elevator();
    potentiometer = new Potentiometer();
    telemetry = new Telemetry();
    intake = new Intake();
    elevatorPivot = new ElevatorPivot();

    // Enabled Feedforward on Elevator
    // elevator.setDefaultCommand(new ElevatorFFCommand(elevator));

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

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

    // Configure the button bindings
    configureButtonBindings();
    // configureTestButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureTestButtonBindings() {
    pilot
        .povUp()
        .whileTrue(new InstantCommand(() -> claw.setWrist(2)))
        .whileFalse(new InstantCommand(() -> claw.setWrist(0)));

    pilot
        .povDown()
        .whileTrue(new InstantCommand(() -> claw.setWrist(-2)))
        .whileFalse(new InstantCommand(() -> claw.setWrist(0)));

    // controller
    //     .povUp()
    //     .whileTrue(new ClawPIDCommand(50.0, claw))
    //     .whileFalse(new ClawFFCommand(claw));
    // controller
    //     .povLeft()
    //     .whileTrue(new ClawPIDCommand(0.0, claw))
    //     .whileFalse(new ClawFFCommand(claw));
    // controller
    //     .povDown()
    //     .whileTrue(new ClawPIDCommand(-40.0, claw))
    //     .whileFalse(new ClawFFCommand(claw));

    // controller
    //     .povUp()
    //     .whileTrue(new InstantCommand(() -> elevatorPivot.setVoltageTunable()))
    //     .whileFalse(new InstantCommand(() -> elevatorPivot.setVoltage(0)));

    // controller.rightTrigger().whileTrue(new ElevatorPIDCommand(30.0, elevator));
    // // .whileFalse(new InstantCommand(() -> claw.setMotor(0)));
    // controller
    //     .leftTrigger()
    //     .onTrue(new InstantCommand(() -> elevator.setMotorVoltage(-3)))
    //     .whileFalse(new ElevatorFFCommand(elevator));

    // ELEVATOR START
    /*
        controller
            .x()
            .whileTrue(
                new ParallelCommandGroup(
                    new ElevatorPIDCommand(70.0, elevator), new ClawPIDCommand(50.0, claw)))
            .whileFalse(
                new InstantCommand(
                    () -> {
                      if (!controller.a().getAsBoolean()
                          && !controller.b().getAsBoolean()
                          && !controller.y().getAsBoolean()
                          && !controller.x().getAsBoolean()) {
                        new ElevatorFFCommand(elevator).schedule();
                        new ClawFFCommand(claw).schedule();
                      }
                    }));
        controller
            .y()
            .whileTrue(new ElevatorPIDCommand(60.0, elevator))
            .whileFalse(
                new InstantCommand(
                    () -> {
                      if (!controller.a().getAsBoolean()
                          && !controller.b().getAsBoolean()
                          && !controller.y().getAsBoolean()
                          && !controller.x().getAsBoolean()) {
                        new ElevatorFFCommand(elevator).schedule();
                      }
                    }));
        controller
            .b()
            .whileTrue(new ElevatorPIDCommand(30.0, elevator))
            .whileFalse(
                new InstantCommand(
                    () -> {
                      if (!controller.a().getAsBoolean()
                          && !controller.b().getAsBoolean()
                          && !controller.y().getAsBoolean()
                          && !controller.x().getAsBoolean()) {
                        new ElevatorFFCommand(elevator).schedule();
                      }
                    }));
        controller
            .a()
            .whileTrue(new ElevatorPIDCommand(0.0, elevator))
            .whileFalse(
                new InstantCommand(
                    () -> {
                      if (!controller.a().getAsBoolean()
                          && !controller.b().getAsBoolean()
                          && !controller.y().getAsBoolean()
                          && !controller.x().getAsBoolean()) {
                        new ElevatorFFCommand(elevator).schedule();
                      }
                    }));
        // new ElevatorFFCommand(elevator));
    */

    pilot
        .rightBumper()
        .whileTrue(new InstantCommand(() -> intake.setFunnelVoltage(2)))
        .whileFalse(new InstantCommand(() -> intake.setFunnelVoltage(0)));

    pilot
        .leftBumper()
        .whileTrue(new InstantCommand(() -> intake.setFunnelVoltage(-2)))
        .whileFalse(new InstantCommand(() -> intake.setFunnelVoltage(0)));

    // controller
    //     .y()
    //     .onTrue(new ElevatorPID(5, elevator))
    //     .onFalse(new InstantCommand(() -> claw.setClaw(0)));
    // controller.a().whileTrue(new ElevatorFFCommand(elevator));
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> pilot.getLeftY() * kDriveSwerveMultipler,
            () -> pilot.getLeftX() * kDriveSwerveMultipler,
            () -> -pilot.getRightX() * kRotationSwerveMultipler,
            () -> mSwerveFieldOriented));

    pilot.y().onTrue(new InstantCommand(() -> mSwerveFieldOriented = !mSwerveFieldOriented));

    // Lock to 0° when A button is held
    pilot
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> pilot.getLeftY() * kDriveSwerveMultipler,
                () -> pilot.getLeftX() * kDriveSwerveMultipler,
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when X button is pressed
    // (I'm used to it on the X button)
    pilot
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    copilot
        .a()
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(true, 0.0, elevator), new ClawPIDCommand(true, 0.0, claw)))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));
    copilot
        .y()
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.L4, elevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.L4, claw)))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));
    // copilot
    //     .a()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new ElevatorPIDCommand(ElevatorConstants.Positions.SCORE, elevator),
    //             new ClawPIDCommand(ClawConstants.Wrist.Positions.SCORE, claw)))
    //     .onFalse(
    //         new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));
    copilot
        .b()
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.PREINTAKE, elevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, claw)))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));

    copilot
        .x()
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, elevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, claw),
                new InstantCommand(
                    () -> claw.setClaw(ClawConstants.Wrist.ClawRollerVolt.INTAKE_CORAL)),
                new InstantCommand(() -> intake.setFunnelVoltage(-1))))
        .onFalse(
            new ParallelCommandGroup(
                new ElevatorFFCommand(elevator),
                new ClawFFCommand(claw),
                new InstantCommand(() -> claw.setClaw(0.0)),
                new InstantCommand(() -> intake.setFunnelVoltage(0))));

    copilot
        .povDown()
        .whileTrue(new InstantCommand(() -> elevatorPivot.setVoltage(-6)))
        .whileFalse(new InstantCommand(() -> elevatorPivot.setVoltage(0)));
    copilot
        .povUp()
        .whileTrue(new InstantCommand(() -> elevatorPivot.setVoltage(6)))
        .whileFalse(new InstantCommand(() -> elevatorPivot.setVoltage(0)));

    copilot
        .leftTrigger()
        .whileTrue(
            new InstantCommand(() -> claw.setClaw(ClawConstants.Wrist.ClawRollerVolt.INTAKE_ALGAE)))
        .whileFalse(new InstantCommand(() -> claw.setClaw(0.0)));
    copilot
        .rightTrigger()
        .whileTrue(
            new InstantCommand(() -> claw.setClaw(ClawConstants.Wrist.ClawRollerVolt.OUTTAKE_REEF)))
        .whileFalse(new InstantCommand(() -> claw.setClaw(0.0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
