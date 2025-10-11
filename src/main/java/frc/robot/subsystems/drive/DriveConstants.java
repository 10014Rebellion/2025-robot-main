package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.controllers.ManualTeleopController.DriverProfiles;

public class DriveConstants {
    ///////////////////// DRIVE BASE \\\\\\\\\\\\\\\\\\\\\\\
    /* PHYSICAL CONSTANTS */
    public static final double kRobotWidthXMeters = Units.inchesToMeters(35);
    public static final double kRobotWidthYMeters = Units.inchesToMeters(37);
    public static final double kTrackWidthXMeters = Units.inchesToMeters(25);
    public static final double kTrackWidthYMeters = Units.inchesToMeters(27);
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0)
      };
    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final double kDrivebaseRadiusMeters =
        Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);
    
    /* DRIVEBASE CONSTRAINTS */
    public static final double kMaxLinearSpeedMPS = 4.5;
    public static final double kMaxLinearAccelerationMPSS = 12.0;

    public static final double kMaxRotationSpeedRadiansPS = Math.toRadians(360);
    public static final double kMaxRotationAccelRadiansPS = Math.toRadians(360) * 10;

    public static final double kMaxAzimuthAngularRadiansPS = Math.toRadians(1200);

    /* Plugged into setpoint generator */
    public static final PathConstraints kAutoDriveConstraints = new PathConstraints(kMaxLinearSpeedMPS, kMaxLinearAccelerationMPSS, kMaxRotationSpeedRadiansPS, kMaxRotationAccelRadiansPS);
    public static final PathConstraints kHyperDriveConstraints = new PathConstraints(6.0, 15.0, Math.toRadians(360), Math.toRadians(360) * 10);
    public static final PathConstraints kFastDriveConstraints = new PathConstraints(6.0, 15.0, Math.toRadians(180), Math.toRadians(180) * 10);
    public static final PathConstraints kMediumDriveConstraints = new PathConstraints(3.0, 8.0, Math.toRadians(180), Math.toRadians(180) * 10);
    public static final PathConstraints kSlowDriveConstraints = new PathConstraints(1.0, 3.0, Math.toRadians(180), Math.toRadians(180) * 10);


    /* MISC */
    public static final double kDriftRate = RobotBase.isReal() ? 2.5 : 5.57;
    public static final double kSniperSpeed = 0.2;

    public static final boolean kDoExtraLogging = false;

    public static final PIDConstants kPPTranslationPID = new PIDConstants(1.0, 0.0, 0.01);
    public static final PIDConstants kPPRotationPID = new PIDConstants(0.8, 0.0, 0.0);

    ///////////////////// MODULES \\\\\\\\\\\\\\\\\\\\\\\
    /* GENERAL SWERVE MODULE CONSTANTS */
    public static final boolean kTurnMotorInvert = false;
    public static final double kAzimuthMotorGearing = 1;
    public static final double kDriveMotorGearing = 4.29 / 1.0;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(1.4175);
    // POSSIBLE CAUSE OF ODOMETRY ISSUE: (Intended)1.5/(measured)1.4175 = 1.05
    public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;

    public static final double kPeakVoltage = 12.0;

    public static final double kDriveStatorAmpLimit = 80.0;
    public static final double kDriveFOCAmpLimit = 80.0;
    public static final double kDriveSupplyAmpLimit = 80.0;

    public static final double kAzimuthStatorAmpLimit = 40.0;
    public static final double kAzimuthFOCAmpLimit = -30.0;

    public static final ModuleControlConfig kModuleControllerConfigs = RobotBase.isReal() ? 
        new ModuleControlConfig(
            new PIDController(100.0, 0.0, 0.0), new SimpleMotorFeedforward(7.0, 0.0, 0.0),
            new PIDController(9.5, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0, 0.0)) :
        new ModuleControlConfig(
            new PIDController(0.1, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 2.36, 0.005), 
            new PIDController(4.5, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0));

            
    /* MODULE SPECIFIC CONSTANTS */
    /* If 180 was added, the person who got the offset had the bevel gears on the wrong side when he did it */
    public static final ModuleHardwareConfig kFrontLeftHardware =
        new ModuleHardwareConfig(
            12, 
            22, 
            22,
            0.971);

    public static final ModuleHardwareConfig kFrontRightHardware =
        new ModuleHardwareConfig(
            11, 
            21, 
            31, 
            -0.432);

    public static final ModuleHardwareConfig kBackLeftHardware =
        new ModuleHardwareConfig(
            13, 
            23, 
            23,
            0.065);

    public static final ModuleHardwareConfig kBackRightHardware =
        new ModuleHardwareConfig(
            14, 
            24, 
            24,
            0.454);

    public static final DriverProfiles defaultProfile =  new DriverProfiles(
        kMaxLinearSpeedMPS, 
        kMaxLinearAccelerationMPSS, 
        1, 
        1, 
        0.075, 
        kMaxRotationSpeedRadiansPS, 
        kMaxRotationAccelRadiansPS, 
        1.0, 
        1.0, 
        0.1, 
        0.2);

    public static final DriverProfiles kBosco =  new DriverProfiles(
            kMaxLinearSpeedMPS, 
            kMaxLinearAccelerationMPSS, 
            1, 
            1, 
            0.075, 
            kMaxRotationSpeedRadiansPS, 
            kMaxRotationAccelRadiansPS, 
            1.0, 
            1.0, 
            0.1, 
            0.2);

    public static final DriverProfiles kEli =  new DriverProfiles(
        kMaxLinearSpeedMPS, 
        kMaxLinearAccelerationMPSS, 
        1, 
        1, 
        0.075, 
        kMaxRotationSpeedRadiansPS, 
        kMaxRotationAccelRadiansPS, 
        1.0, 
        1.0, 
        0.1, 
        0.2);

    public static record ModuleHardwareConfig(
        int driveID, int azimuthID, int encoderID, double offset) {}
    
    public static record ModuleControlConfig(
        PIDController driveController,
        SimpleMotorFeedforward driveFF,
        PIDController azimuthController,
        SimpleMotorFeedforward azimuthFF) {}
}