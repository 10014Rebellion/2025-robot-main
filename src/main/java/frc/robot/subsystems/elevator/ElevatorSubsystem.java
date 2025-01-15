package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ConfigMotor;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax mElevatorMotor;
    private final RelativeEncoder mElevatorEncoder;

    public ElevatorSubsystem() {
        mElevatorMotor = new SparkMax(ElevatorConstants.kMotorId, MotorType.kBrushless);
        mElevatorEncoder = mElevatorMotor.getEncoder();
        ConfigMotor.configSparkMax(
            mElevatorMotor, ElevatorConstants.kInverted,
            ElevatorConstants.kCurrentLimit, ElevatorConstants.kIdleMode,
            mElevatorEncoder, ElevatorConstants.kEncConversionFactor, ElevatorConstants.kEncCPR
        );
    }

    public void setVoltage(double pVolts) {
        setVoltage(pVolts, false);
    }

    public void setPercentOutput(double pPercent) {
        setPercentOutput(pPercent, false);
    }

    public void setVoltage(double pVolts, boolean pOverride) {
        if (canMove(pVolts) || pOverride)
            mElevatorMotor.setVoltage(MathUtil.clamp(pVolts, -12, 12));
        else
            stopAllMotors();
    }

    public void setPercentOutput(double pPercent, boolean pOverride) {
        if (canMove(pPercent) || pOverride)
            mElevatorMotor.set(MathUtil.clamp(pPercent, -1, 1));
        else
            stopAllMotors();
    }

    public void stopAllMotors() {
        mElevatorMotor.set(0);
    }

    /////////////// GET MOTOR \\\\\\\\\\\\\\\
    public double getVoltage() {
        return mElevatorMotor.getBusVoltage();
    }

    public double getPercentOutput() {
        return mElevatorMotor.get();
    }

    /////////////// ENCODER \\\\\\\\\\\\\\\
    public double getEncoderPosition() {
        return mElevatorEncoder.getPosition();
    }

    public double getRawEncoderCount() {
        return mElevatorEncoder.getPosition() * ElevatorConstants.kEncCPR
                / ElevatorConstants.kEncConversionFactor;
    }

    private boolean canMove(double pMagnitude) {
        double currentPosition = getRawEncoderCount();
        if (pMagnitude > 0 && currentPosition >= ElevatorConstants.kMaxHeight) {
            System.out.println("!!! ELEVATOR UPPER ANGLE LIMIT REACHED !!!");
            return false;
        }

        if (pMagnitude < 0 && currentPosition <= ElevatorConstants.kMinHeight) {
            System.out.println("!!! ELEVATOR LOWER ANGLE LIMIT REACHED !!!");
            return false;
        }
        return true;
    }

    /////////////// PERIODIC \\\\\\\\\\\\\\\
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder Position", getEncoderPosition());
        SmartDashboard.putNumber("Elevator Encoder Ticks", getRawEncoderCount());
        SmartDashboard.putNumber("Elevator Motor Voltage", getVoltage());
        SmartDashboard.putBoolean("Elevator Can Move", canMove(getEncoderPosition()));

        if (!canMove(getPercentOutput()))
            stopAllMotors();
    }
}
