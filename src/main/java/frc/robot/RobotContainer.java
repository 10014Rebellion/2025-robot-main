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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ExtendOuttake;
import frc.robot.commands.GoToIntake;
import frc.robot.commands.GoToPose;
import frc.robot.commands.ShootAlgae;
import frc.robot.subsystems.LEDs.LEDInterface;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawConstants.Claw.ClawRollerVolt;
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
import frc.robot.subsystems.elevator.ElevatorConstants.Positions;
import frc.robot.subsystems.elevator.ElevatorFFCommand;
import frc.robot.subsystems.elevator.ElevatorPIDCommand;
import frc.robot.subsystems.elevatorPivot.ElevatorPivot;
import frc.robot.subsystems.intake.OTBIntake;
import frc.robot.subsystems.potentiometer.Potentiometer;
import frc.robot.subsystems.telemetry.Telemetry;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
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
  private final Vision vision;
  private final Elevator elevator;
  private final ElevatorPivot pivot;
  private final Potentiometer potentiometer;
  private final Telemetry telemetry;
  private final OTBIntake intake;
  private final LEDInterface LEDs;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(0);
  private final CommandGenericHID operatorButtonboard = new CommandGenericHID(1);

  private final Pose2d targetPose = new Pose2d(1, 0.5, new Rotation2d(45));

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
    intake = new OTBIntake();
    pivot = new ElevatorPivot();

    LEDs = new LEDInterface();

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

    vision = new Vision(() -> drive.getRotation(), () -> drive.getModulePositions());

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
  }

  private void configureTestButtonBindings() {
    driverController.povUp()
        .whileTrue(new InstantCommand(() -> intake.setRightPivot(4)))
        .whileFalse(new InstantCommand(() -> intake.setRightPivot(0)));

    driverController.povDown()
        .whileTrue(new InstantCommand(() -> intake.setRightPivot(-2)))
        .whileFalse(new InstantCommand(() -> intake.setRightPivot(0)));

    driverController.rightBumper()
        .onTrue(new InstantCommand(() -> intake.setRightRoller(12)))
        .onFalse(new InstantCommand(() -> intake.setRightRoller(0)));

    driverController.leftBumper()
        .onTrue(new InstantCommand(() -> intake.setRightRoller(-6)))
        .onFalse(new InstantCommand(() -> intake.setRightRoller(0)));

    operatorController
        .x()
        .whileTrue(new GoToIntake(elevator, claw))
        .whileFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));
    operatorController
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(Positions.POSTINTAKE, elevator),
                new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, claw),
                new InstantCommand(() -> claw.setClaw(ClawRollerVolt.INTAKE_CORAL))))
        .whileFalse(
            new ParallelCommandGroup(
                new ElevatorFFCommand(elevator),
                new ClawFFCommand(claw),
                new InstantCommand(() -> claw.setClaw(0))));
    operatorController
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> claw.setClaw(-1)))
        .whileFalse(new InstantCommand(() -> claw.setClaw(0)));
    operatorController
        .leftTrigger()
        .whileTrue(new InstantCommand(() -> claw.setClaw(1)))
        .whileFalse(new InstantCommand(() -> claw.setClaw(0)));
    operatorController
        .rightBumper()
        .whileTrue(new ShootAlgae(claw))
        .whileFalse(new InstantCommand(() -> claw.setClaw(0)));
    operatorController
        .povUp()
        .onTrue(
            new ExtendOuttake(
                elevator,
                claw,
                drive,
                ElevatorConstants.Positions.L4,
                ClawConstants.Wrist.Positions.L4))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));
    operatorController
        .povLeft()
        .onTrue(
            new ExtendOuttake(
                elevator,
                claw,
                drive,
                ElevatorConstants.Positions.L3,
                ClawConstants.Wrist.Positions.L3))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));
    operatorController
        .povRight()
        .onTrue(
            new ExtendOuttake(
                elevator,
                claw,
                drive,
                ElevatorConstants.Positions.L2,
                ClawConstants.Wrist.Positions.L2))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));
    operatorController
        .povRight()
        .onTrue(
            new ExtendOuttake(
                elevator,
                claw,
                drive,
                ElevatorConstants.Positions.L1,
                ClawConstants.Wrist.Positions.L1))
        .onFalse(
            new ParallelCommandGroup(new ElevatorFFCommand(elevator), new ClawFFCommand(claw)));
    driverController.povRight()
        .whileTrue(new InstantCommand(() -> pivot.setVoltage(12)))
        .whileFalse(new InstantCommand(() -> pivot.setVoltage(0)));
    driverController.povLeft()
        .whileTrue(new InstantCommand(() -> pivot.setVoltage(-12)))
        .whileFalse(new InstantCommand(() -> pivot.setVoltage(0)));
    driverController.b().whileTrue(new ElevatorPIDCommand(true, 0, elevator));
  }

  private void configureButtonBindings() {
    // operatorButtonboard.button(ButtonBoardConstants.Button1)
    //     .onTrue(new InstantCommand())
    //     .onFalse(new InstantCommand());
    
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> mSwerveFieldOriented));

    driverController.b().onTrue(new InstantCommand(() -> mSwerveFieldOriented = !mSwerveFieldOriented));

    driverController.rightTrigger()
        .whileTrue(new GoToPose(() -> targetPose, () -> drive.getPose(), drive));
    driverController.leftTrigger()
        .whileTrue(
            new GoToPose(() -> new Pose2d(0, 0, new Rotation2d(0)), () -> drive.getPose(), drive));

    driverController.povUp()
        .whileTrue(
            new GoToPose(
                () -> vision.getReefScoringPose(7, 10, VisionConstants.PoseOffsets.CENTER),
                () -> vision.getPose(),
                drive));

    driverController.povLeft()
        .whileTrue(
            new GoToPose(
                () -> vision.getReefScoringPose(7, 10, VisionConstants.PoseOffsets.LEFT),
                () -> vision.getPose(),
                drive));

    driverController.povRight()
        .whileTrue(
            new GoToPose(
                () -> vision.getReefScoringPose(7, 10, VisionConstants.PoseOffsets.RIGHT),
                () -> vision.getPose(),
                drive));

    // Reset gyro to 0° when X button is pressed
    driverController.x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  // public void configureCopilotBindings() {
  // copilot
  // .button(ControllerConstants.kL4Button)
  // .onTrue(
  // new ParallelCommandGroup(
  // new ElevatorPIDCommand(ElevatorConstants.Positions.L4, elevator),
  // new ClawPIDCommand(ClawConstants.Wrist.Positions.L4, claw)));
  // copilot
  // .button(ControllerConstants.kL3Button)
  // .onTrue(
  // new ParallelCommandGroup(
  // new ElevatorPIDCommand(ElevatorConstants.Positions.L3, elevator),
  // new ClawPIDCommand(ClawConstants.Wrist.Positions.L3, claw)));
  // copilot
  // .button(ControllerConstants.kL2Button)
  // .onTrue(
  // new ParallelCommandGroup(
  // new ElevatorPIDCommand(ElevatorConstants.Positions.L2, elevator),
  // new ClawPIDCommand(ClawConstants.Wrist.Positions.L2, claw)));
  // copilot
  // .button(ControllerConstants.kL1Button)
  // .onTrue(
  // new ParallelCommandGroup(
  // new ElevatorPIDCommand(ElevatorConstants.Positions.L1, elevator),
  // new ClawPIDCommand(ClawConstants.Wrist.Positions.L1, claw)));

  // // This one probably needs a lot more work
  // copilot
  // .button(ControllerConstants.kIntakeCoralButton)
  // .onTrue(
  // new ParallelCommandGroup(
  // new ElevatorPIDCommand(ElevatorConstants.Positions.POSTINTAKE, elevator),
  // new ClawPIDCommand(ClawConstants.Wrist.Positions.INTAKE, claw)));
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
