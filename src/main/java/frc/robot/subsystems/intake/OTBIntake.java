package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OTBIntake extends SubsystemBase{
    private SparkFlex mRightPivotMotor;
    private SparkFlex mRightRollerMotor;

    private AbsoluteEncoder mRightPivotEncoder;

    public OTBIntake() {
        this.mRightPivotMotor = new SparkFlex(IntakeConstants.OTBIntake.kRightPivotID, IntakeConstants.OTBIntake.kMotorType);
        this.mRightRollerMotor = new SparkFlex(IntakeConstants.OTBIntake.kRightRollerID, IntakeConstants.OTBIntake.kMotorType);

        mRightPivotMotor.configure(IntakeConstants.OTBIntake.kRightPivotConfig,ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

        mRightRollerMotor.configure(IntakeConstants.OTBIntake.kRollerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

        // Note: it will have an encoder, just not right this second. 
        // This is because the pivot motor is a sparkflex here, not a sparkmax, so it cant be called normally
        //mRightPivotEncoder = ;

    }

    public void setRightRoller(double pVoltage) {
        mRightRollerMotor.setVoltage(filterVoltage(pVoltage));
    }

    public void setRightPivot(double pVoltage) {
        mRightPivotMotor.setVoltage(filterVoltage(pVoltage));
    }

    private double filterVoltage(double pVoltage) {
        return MathUtil.clamp(pVoltage, -12.0, 12.0);
    }

    public double getEncoderMeasurement() {
        // No encoder yet. fix this later
        // TO DO:
        return 0.0;
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Intake/Right/Pivot Position", getEncoderMeasurement());
        SmartDashboard.putNumber("Intake/Right/Pivot Current", mRightPivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Right/Roller Current", mRightRollerMotor.getOutputCurrent());

    }
}

