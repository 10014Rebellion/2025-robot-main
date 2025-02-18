// // Copyright (c) 2025 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by an MIT-style
// // license that can be found in the LICENSE file at
// // the root directory of this project.

// package org.littletonrobotics.frc2025.commands;

// import static
// org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import java.util.Arrays;
// import org.littletonrobotics.frc2025.Constants;
// import org.littletonrobotics.frc2025.RobotState;
// import org.littletonrobotics.frc2025.subsystems.drive.Drive;
// import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
// import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
// import org.littletonrobotics.frc2025.util.trajectory.HolonomicTrajectory;
// import frc.robot.util.trajectory.TrajectoryGenerationHelpers;
// import org.littletonrobotics.junction.Logger;

// @ExtensionMethod({TrajectoryGenerationHelpers.class})
// public class DriveTrajectory extends Command {
//   private static final LoggedTunableNumber linearkP =
//       new LoggedTunableNumber("DriveTrajectory/LinearkP");
//   private static final LoggedTunableNumber linearkD =
//       new LoggedTunableNumber("DriveTrajectory/LinearkD");
//   private static final LoggedTunableNumber thetakP =
//       new LoggedTunableNumber("DriveTrajectory/ThetakP");
//   private static final LoggedTunableNumber thetakD =
//       new LoggedTunableNumber("DriveTrajectory/thetakD");

//   static {
//     switch (Constants.getRobot()) {
//       case COMPBOT, DEVBOT -> {
//         linearkP.initDefault(8.0);
//         linearkD.initDefault(0.0);
//         thetakP.initDefault(4.0);
//         thetakD.initDefault(0.0);
//       }
//       default -> {
//         linearkP.initDefault(4.0);
//         linearkD.initDefault(0.0);
//         thetakP.initDefault(4.0);
//         thetakD.initDefault(0.0);
//       }
//     }
//   }

//   private final Drive drive;
//   private final HolonomicTrajectory trajectory;
//   private final Timer timer = new Timer();

//   private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
//   private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
//   private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

//   public DriveTrajectory(Drive drive, HolonomicTrajectory trajectory) {
//     this.drive = drive;
//     this.trajectory = trajectory;

//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
//     addRequirements(drive);
//   }

//   @Override
//   public void initialize() {
//     timer.restart();

//     xController.reset();
//     yController.reset();
//     thetaController.reset();

//     Logger.recordOutput(
//         "Trajectory/TrajectoryPoses",
//         Arrays.stream(trajectory.getTrajectoryPoses())
//             .map(AllianceFlipUtil::apply)
//             .toArray(Pose2d[]::new));
//   }

//   @Override
//   public void execute() {
//     // Update tunable numbers
//     if (linearkP.hasChanged(hashCode()) || linearkD.hasChanged(hashCode())) {
//       xController.setPID(linearkP.get(), 0.0, linearkD.get());
//       yController.setPID(linearkP.get(), 0.0, linearkD.get());
//     }
//     if (thetakP.hasChanged(hashCode()) || thetakD.hasChanged(hashCode())) {
//       thetaController.setPID(thetakP.get(), 0.0, thetakD.get());
//     }

//     Pose2d robot = RobotState.getInstance().getEstimatedPose();
//     // Get setpoint state and flip
//     VehicleState setpointState = AllianceFlipUtil.apply(trajectory.sample(timer.get()));

//     // Calculate feedback
//     double xFeedback = xController.calculate(robot.getX(), setpointState.getX());
//     double yFeedback = yController.calculate(robot.getY(), setpointState.getY());
//     double thetaFeedback =
//         thetaController.calculate(robot.getRotation().getRadians(), setpointState.getTheta());

//     // Calculate module forces
//     var moduleForces =
//         setpointState.getModuleForcesList().stream()
//             .map(
//                 forces ->
//                     new Translation2d(forces.getFx(), forces.getFy())
//                         .rotateBy(Rotation2d.fromRadians(setpointState.getTheta()).unaryMinus())
//                         .toVector())
//             .toList();

//     // Command drive
//     drive.runVelocity(
//         ChassisSpeeds.fromFieldRelativeSpeeds(
//             xFeedback + setpointState.getVx(),
//             yFeedback + setpointState.getVy(),
//             thetaFeedback + setpointState.getOmega(),
//             Rotation2d.fromDegrees(robot.getRotation().getDegrees())),
//         moduleForces);

//     // Log outputs
//     Logger.recordOutput("Trajectory/RobotPose", robot);
//     Logger.recordOutput("Trajectory/SetpointPose", setpointState.getPose());
//     Logger.recordOutput(
//         "Trajectory/Feedback",
//         new Pose2d(xFeedback, yFeedback, Rotation2d.fromRadians(thetaFeedback)));
//     Logger.recordOutput(
//         "Trajectory/VelocityFeedforward",
//         new Pose2d(
//             setpointState.getVx(),
//             setpointState.getVy(),
//             Rotation2d.fromRadians(setpointState.getOmega())));
//   }

//   @Override
//   public void end(boolean interrupted) {}

//   @Override
//   public boolean isFinished() {
//     return timer.get() >= trajectory.getDuration();
//   }
// }
