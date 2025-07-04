package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private DCMotorSim driveMotor = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, kDriveMotorGearing), 
            DCMotor.getKrakenX60Foc(1), 0.0, 0.0);
    private DCMotorSim azimuthMotor = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025, 52), 
            DCMotor.getKrakenX60Foc(1), 0.0, 0.0);

    private double driveAppliedVolts = 0.0;
    private double azimuthAppliedVolts = 0.0;

    private PIDController drivePID = kModuleControllerConfigs.driveController();

    private PIDController azimuthPID = kModuleControllerConfigs.azimuthController();

    public ModuleIOSim() {
        azimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        driveMotor.update(0.02);
        azimuthMotor.update(0.02);

        inputs.drivePositionM = driveMotor.getAngularPositionRotations() * kWheelCircumferenceMeters;
        inputs.driveVelocityMPS = (driveMotor.getAngularVelocityRPM() * kWheelCircumferenceMeters) / 60.0;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveStatorCurrentAmps = Math.abs(driveMotor.getCurrentDrawAmps());
        inputs.driveTemperatureCelsius = 0.0;
        inputs.azimuthAppliedVolts = azimuthAppliedVolts;
        inputs.azimuthMotorVolts = azimuthAppliedVolts;

        inputs.azimuthAbsolutePosition = new Rotation2d(azimuthMotor.getAngularPositionRad());
        inputs.azimuthPosition = new Rotation2d(azimuthMotor.getAngularPositionRad());
        inputs.azimuthVelocity = Rotation2d.fromRadians(azimuthMotor.getAngularVelocityRadPerSec());
        inputs.azimuthStatorCurrentAmps = Math.abs(azimuthMotor.getCurrentDrawAmps());
        inputs.azimuthTemperatureCelsius = 0.0;
        inputs.azimuthAppliedVolts = azimuthAppliedVolts;
        inputs.azimuthMotorVolts = azimuthAppliedVolts;
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        /* sets drive voltage between -kPeakVoltage and kPeakVoltage */
        driveAppliedVolts = MathUtil.clamp(volts, -kPeakVoltage, kPeakVoltage);
        driveMotor.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setDriveVelocity(double velocityMPS, double feedforward) {
        /* Sets drive velocity using PID */
        setDriveVolts(
            drivePID.calculate(
                driveMotor.getAngularVelocityRPM() * kWheelCircumferenceMeters / 60, 
                velocityMPS) 
            + feedforward);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        /* Sets drive velocity PID */
        drivePID.setPID(kP, kI, kD);
    }

    @Override
    public void resetAzimuthEncoder() {
        /* No code is needed, sim doesn't need an implementation of this */
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        /* sets azimuth voltage between -kPeakVoltage and kPeakVoltage */
        azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        azimuthMotor.setInputVoltage(azimuthAppliedVolts);
    }

    @Override
    public void setAzimuthPosition(Rotation2d position, double feedforward) {
        /* Sets azimuth position using PID */
        setAzimuthVolts(azimuthPID.calculate(azimuthMotor.getAngularPositionRad(), position.getRadians()) + feedforward);
    }

    @Override
    public void setAzimuthPID(double kP, double kI, double kD) {
        /* Sets azimuth position PID */
        azimuthPID.setPID(kP, kI, kD);
    }
}
