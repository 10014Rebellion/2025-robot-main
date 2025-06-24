package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.telemetry.Elastic;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  public static boolean gIsBlueAlliance;
  // private String autoName, newAutoName;
  Optional<Alliance> ally = DriverStation.getAlliance();
  Optional<Alliance> newAlly;

  private static void updateAlliance() {
    gIsBlueAlliance = DriverStation.getAlliance().isPresent()
        ? DriverStation.getAlliance().get().equals(Alliance.Blue)
        : true;
  }

  public Robot() {
    updateAlliance();

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    updateAlliance();

    SmartDashboard.putBoolean("Alliance", gIsBlueAlliance);

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // ally = DriverStation.getAlliance();
    // newAutoName = robotContainer.getAutonomousCommand().getName();
    // if (autoName != newAutoName | ally != newAlly) {
    //   newAlly = ally;
    //   autoName = newAutoName;
    //   if (AutoBuilder.getAllAutoNames().contains(autoName)) {
    //     System.out.println("Displaying " + autoName);
    //     try {
    //       List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    //       List<Pose2d> poses = new ArrayList<>();
    //       for (PathPlannerPath path : pathPlannerPaths) {
    //         if (gIsBlueAlliance) {
    //           poses.addAll(
    //               path.getAllPathPoints().stream()
    //                   .map(
    //                       point -> new Pose2d(
    //                           point.position.getX(), point.position.getY(), new Rotation2d()))
    //                   .collect(Collectors.toList()));
    //         } else if (ally.get() == Alliance.Red) {
    //           poses.addAll(
    //               path.getAllPathPoints().stream()
    //                   .map(
    //                       point -> new Pose2d(
    //                           PoseConstants.WeldedField.kFieldLengthM - point.position.getX(),
    //                           PoseConstants.WeldedField.kFieldWidthM - point.position.getY(),
    //                           new Rotation2d()))
    //                   .collect(Collectors.toList()));
    //         }
    //       }

    //       robotContainer.getTelemetry().getAutonPreviewField().getObject("path").setPoses(poses);
    //       robotContainer
    //           .getTelemetry()
    //           .updateAutonFieldPose(
    //               new Pose2d(
    //                   poses.get(0).getX(),
    //                   poses.get(0).getY(),
    //                   gIsBlueAlliance ? Rotation2d.k180deg : Rotation2d.kZero));
    //     } catch (IOException e) {
    //       e.printStackTrace();
    //     } catch (Exception e) {
    //       if (e instanceof ParseException) {
    //         e.printStackTrace();
    //       } else {
    //         e.printStackTrace();
    //       }
    //     }
    //   }
    // }
    // // Pose2d pose = robotContainer.getDrivetrain().getPose();
    // // robotContainer.getTelemetry().add(pose);
    // SmartDashboard.putData(robotContainer.getTelemetry().getAutonPreviewField());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    Elastic.selectTab("Autonomous");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.initTriggers();

    Elastic.selectTab("Teleoperated");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
