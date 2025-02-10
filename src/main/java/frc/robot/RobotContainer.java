package frc.robot;

import java.util.List;
import java.util.ArrayList;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;

import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.VisionNew;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.PoseCamera;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Axes Multipler
    private final double kDriveSwerveMultipler = 1;
    private final double kRotationSwerveMultipler = 0.8;

    // Subsystems
    private final Drive drive;
    private final Claw claw;
    private final Elevator elevator;
    private final VisionNew vision;

    // Vision (Imma kill someone)
    private final List<PoseCamera> cameras;


    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Field Oriented
    private boolean mSwerveFieldOriented = true;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        claw = new Claw();
        elevator = new Elevator();
        vision = new VisionNew();

        cameras = new ArrayList<PoseCamera> ();

        VisionConstants.cameraPositions.forEach((cameraName, cameraTransform) -> {
            cameras.add(new PoseCamera(
              cameraName, 
              cameraTransform, 
              VisionConstants.kPoseStrategy, 
              VisionConstants.kFallbackPoseStrategy, 
              VisionConstants.kAprilTagFieldLayout
            ));
          });

        

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
        controller
            .povUp()
            .onTrue(new InstantCommand(() -> claw.setMotor(2)))
            .onFalse(new InstantCommand(() -> claw.setMotor(0)));

        controller
            .povDown()
            .onTrue(new InstantCommand(() -> claw.setMotor(-2)))
            .onFalse(new InstantCommand(() -> claw.setMotor(0)));

        controller
            .rightTrigger()
            .onTrue(new InstantCommand(() -> elevator.setMotor(3)))
            .onFalse(new InstantCommand(() -> elevator.setMotor(0)));

        controller
            .leftTrigger()
            .onTrue(new InstantCommand(() -> elevator.setMotor(-3)))
            .onFalse(new InstantCommand(() -> elevator.setMotor(0)));

        controller
            .x()
            .onTrue(new InstantCommand(() -> claw.setClaw(1)))
            .onFalse(new InstantCommand(() -> claw.setClaw(0)));

        controller
            .b()
            .onTrue(new InstantCommand(() -> claw.setClaw(-1)))
            .onFalse(new InstantCommand(() -> claw.setClaw(0)));
    }

    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> controller.getLeftY() * kDriveSwerveMultipler,
                () -> controller.getLeftX() * kDriveSwerveMultipler,
                () -> -controller.getRightX() * kRotationSwerveMultipler,
                () -> mSwerveFieldOriented));

        controller.y().onTrue(new InstantCommand(() -> mSwerveFieldOriented = !mSwerveFieldOriented));

        // Lock to 0° when A button is held
        controller
            .a()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> controller.getLeftY() * kDriveSwerveMultipler,
                    () -> controller.getLeftX() * kDriveSwerveMultipler,
                    () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
            .b()
            .onTrue(
                Commands.runOnce(
                        () ->
                            drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                        drive)
                    .ignoringDisable(true));
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
