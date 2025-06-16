// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double kHighSpeedTrans = 1.0;
  public static final double kHighSpeedRot = 1.0;
  public static final double kLowSpeedTrans = .3;
  public static final double kLowSPeedRot = .25;

  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(25);
  public static final double wheelBase = Units.inchesToMeters(27);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };
  public static final SwerveDriveKinematics kSwerveDriveKinematics =
      new SwerveDriveKinematics(moduleTranslations);

  // Zeroed rotation values for each module, see setup instructions
  // public static final Rotation2d frontLeftZeroRotation = new
  // Rotation2d(0.2009058);
  // public static final Rotation2d frontRightZeroRotation = new
  // Rotation2d(0.8087860);
  // public static final Rotation2d backLeftZeroRotation = new
  // Rotation2d(0.2965502);
  // public static final Rotation2d backRightZeroRotation = new
  // Rotation2d(0.7473245);

  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(3.0 * Math.PI / 2.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2.0);

  // Device CAN IDs
  public static final int pigeonCanId = 2;

  // Intake is on the right, we want to make it so the left side is the front side
  //   public static final int frontRightDriveCanId = 11;
  //   public static final int frontLeftDriveCanId = 15; // WAS 14
  //   public static final int backRightDriveCanId = 12;
  //   public static final int backLeftDriveCanId = 13;

  //   public static final int frontRightTurnCanId = 21;
  //   public static final int frontLeftTurnCanId = 24;
  //   public static final int backRightTurnCanId = 22;
  //   public static final int backLeftTurnCanId = 23;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (13.0 * 15.0); // MAXSwerve with 13 pinion teeth
  // and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.005;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.00;
  public static final double driveKv = 0.086; // 0.083 gave within 0.01 on most velocity
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Whole Bot PID
  public static final double drivebaseDriveKp = 1;
  public static final double drivebaseDriveKd = 0.001;

  public static final double drivebaseThetaKp = 2.0;
  public static final double drivebaseThetaKd = 0.0;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.49;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 0.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double kTotalWidthM = Units.inchesToMeters(35.5);
  public static final double kTotalLengthM = Units.inchesToMeters(38);

  public static final double robotMassKg = 61.9;
  public static final double robotMOI = 6.883; // TODO: CONFIGURE ME MAY BE 5.39
  public static final double wheelCOF = 1.6;

  public static final double kVortexFreeSpeed = 5.33;
  public static final double kMaxLinearSpeedMPS = kVortexFreeSpeed;
  public static final double kMaxAngularSpeedRadPS = kVortexFreeSpeed / driveBaseRadius;

  public static final double kVortexFreeSpeedRPM = 6784;
  public static final double kVortexStallTorqueNM = 3.6;
  public static final double kVortexStallCurrentA = 211;

  public static final double kMaxSwerveGearReduction = 5.08;
  public static final double kWheelRPM = kVortexFreeSpeedRPM / kMaxSwerveGearReduction;
  public static final double kWheelSpeedMPS = (kWheelRPM * 2 * Math.PI * wheelRadiusMeters) / 60;

  public static final double kGearboxEfficiency = 0.95; // estimated
  public static final double kRunningTorque =
      kVortexStallTorqueNM * (driveMotorCurrentLimit / kVortexStallCurrentA);

  public static final double kOutputTorque =
      (kRunningTorque * kMaxSwerveGearReduction) * kGearboxEfficiency;
  public static final double kMaxWheelTorque = kOutputTorque / wheelRadiusMeters;

  public static final double kMaxLinearAccelerationMPSSq = wheelCOF * 9.81; // Mass cancels out
  public static final double kMaxAngularAccelerationRadPSSq = kMaxWheelTorque / robotMOI;

  public static final double kTurnFreeSpeedRPM = 11000;
  public static final double kMaxTurnAngularRadPS =
      (kTurnFreeSpeedRPM / turnMotorReduction) * (2 * Math.PI / 60);

  // Pathplanner constraints
  public static final PathConstraints kDriveConstraints =
      new PathConstraints(
          kMaxLinearSpeedMPS,
          kMaxLinearAccelerationMPSSq,
          kMaxAngularSpeedRadPS,
          kMaxAngularAccelerationRadPSSq);

  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
