package frc.robot.util.swerve;

import edu.wpi.first.math.Vector;

import com.pathplanner.lib.util.DriveFeedforwards;

import static frc.robot.subsystems.drive.DriveConstants.kDriveMotorGearing;
import static frc.robot.subsystems.drive.DriveConstants.kKinematics;
import static frc.robot.subsystems.drive.DriveConstants.kMaxLinearSpeedMPS;
import static frc.robot.subsystems.drive.DriveConstants.kWheelRadiusMeters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.math.EqualsUtil;
import frc.robot.util.pathplanner.SwerveSetpoint;

public class SwerveUtils {
    private static final double dt = 0.02;
    private static final DCMotor kKrakenFOCModel = DCMotor.getKrakenX60Foc(1);
    private static final double kJitterThreshold = 0.01;

    /* 
     * Helps driftRate value helps account for translation drift while rotating and riving
     * Also makes speed go from continous to discrete. Technical details:
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
     */
    public static ChassisSpeeds discretize(ChassisSpeeds speeds, double driftRate) {
        var desiredDeltaPose = new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * driftRate));
        var twist = new Pose2d().log(desiredDeltaPose);

        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    public static SwerveModuleState copyState(SwerveModuleState state) {
        return new SwerveModuleState(state.speedMetersPerSecond, state.angle);
    }

    /* If setpoint is very low keep module at current angle, as it causes jitter trying to control such little movement */
    public static Rotation2d removeAzimuthJitter(SwerveModuleState setpoint, SwerveModuleState current) {
        return Math.abs(setpoint.speedMetersPerSecond / kMaxLinearSpeedMPS) < kJitterThreshold ? current.angle : setpoint.angle;
    }


    public static double projectTorque(SwerveModuleState currentState, Vector<N2> wheelForce) {
        Vector<N2> wheelDirection = VecBuilder.fill(currentState.angle.getCos(), currentState.angle.getSin());
        double wheelForceN = wheelForce.dot(wheelDirection);
        return wheelForceN; 
    }

    // PATHPLANNER TORQUE UTILS \\ 
    /* Torque isn't directionally changed when the velocity flip case happen SwerveModuleState.optimize() */
    public static double optimizeTorque(SwerveModuleState unOptimized, SwerveModuleState optimized, double motorAmperage, int i) {
        return isSpeedOptimized(unOptimized, optimized, i) ? - motorAmperage : motorAmperage;
    }

    /* Check if optimize changed module velocity direction */
    public static boolean isSpeedOptimized(SwerveModuleState state, SwerveModuleState optimizedState, int i) {
        boolean isSpeedOptimized = !EqualsUtil.epsilonEquals(state.speedMetersPerSecond, optimizedState.speedMetersPerSecond);
        Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/isSpeedOptimized", isSpeedOptimized);
        return isSpeedOptimized;
    }

    // ASSUMES THE CHOREO'S MOTOR TORQUE DOESN'T ALREADY EXCEED THE MOTOR'S LIMIT
    public static double convertChoreoNewtonsToAmps(SwerveModuleState currentState, DriveFeedforwards ff, int i) {
        double choreoLinearForceNewtons = projectTorque(currentState, 
            VecBuilder.fill(
                ff.robotRelativeForcesXNewtons()[i], 
                ff.robotRelativeForcesYNewtons()[i]));

        // NEWTONS -> GEARBOX TORQUE -> MOTOR TORQUE
        double driveMotorTorque = (choreoLinearForceNewtons * kWheelRadiusMeters) / kDriveMotorGearing;
        double driveMotorAmperage = kKrakenFOCModel.getCurrent(driveMotorTorque);

        return driveMotorAmperage;
    }

    public static void logDriveFeedforward(DriveFeedforwards ff, int i) {
        Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Acceleration", ff.accelerationsMPSSq()[i]);
        Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Force", ff.linearForcesNewtons()[i]);
        Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Current", ff.torqueCurrentsAmps()[i]);
    }

    /* Log different variations of the desired swerve module states */
    public static void logPossibleDriveStates(boolean doLogging, ChassisSpeeds desiredSpeeds, SwerveModuleState[] currentStates, SwerveSetpoint previousSetpoint, Rotation2d robotRotation) {
        if(doLogging) {
            /* Regular setpoint generation */
            SwerveModuleState[] unOptimizedSetpointStates = kKinematics.toSwerveModuleStates(desiredSpeeds);
            for(int i = 0; i < 4; i++) {
                unOptimizedSetpointStates[i] = new SwerveModuleState(
                    unOptimizedSetpointStates[i].speedMetersPerSecond,
                    removeAzimuthJitter(unOptimizedSetpointStates[i], currentStates[i]));
                unOptimizedSetpointStates[i].optimize(currentStates[i].angle);
                unOptimizedSetpointStates[i].cosineScale(currentStates[i].angle);
            }
            SwerveDriveKinematics.desaturateWheelSpeeds(unOptimizedSetpointStates, kMaxLinearSpeedMPS);
            Logger.recordOutput("Drive/Swerve/preOptimizedSetpoints", unOptimizedSetpointStates);

            unOptimizedSetpointStates = kKinematics.toSwerveModuleStates(desiredSpeeds);
            for(int i = 0; i < 4; i++) {
                unOptimizedSetpointStates[i] = new SwerveModuleState(
                    unOptimizedSetpointStates[i].speedMetersPerSecond,
                    removeAzimuthJitter(unOptimizedSetpointStates[i], currentStates[i]));
                unOptimizedSetpointStates[i].optimize(currentStates[i].angle);
                unOptimizedSetpointStates[i].cosineScale(currentStates[i].angle);
            }
            Logger.recordOutput("Drive/Swerve/saturatedPreOptimizedSetpoints", unOptimizedSetpointStates);
            Logger.recordOutput("Drive/Odometry/preOptimizedChassisSpeeds", kKinematics.toChassisSpeeds(unOptimizedSetpointStates));

            /* Non-generated swerve setpoints */
            SwerveModuleState[] swerveModuleStates = kKinematics.toSwerveModuleStates(desiredSpeeds);

            Logger.recordOutput("Drive/Swerve/RegularSetpoints", swerveModuleStates);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxLinearSpeedMPS);

            Logger.recordOutput("Drive/Swerve/SaturatedRegularSetpoints", swerveModuleStates);
            Logger.recordOutput("Drive/Odometry/FieldRegularChassisSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(
                kKinematics.toChassisSpeeds(swerveModuleStates), robotRotation));
        }
    }

    public static SwerveModuleState[] zeroStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
        };
    }

    public static double getTorqueOfKrakenDriveMotor(double amps) {
        return kKrakenFOCModel.getTorque(amps);
    }
}