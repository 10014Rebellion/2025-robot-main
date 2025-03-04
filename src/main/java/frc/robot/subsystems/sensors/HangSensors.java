package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSensors extends SubsystemBase {
    DigitalInput mBeamBreakRight;
    DigitalInput mBeamBreakLeft;

    public HangSensors() {
        mBeamBreakRight = new DigitalInput(3);
        mBeamBreakLeft = new DigitalInput(4);
    }

    boolean hasCageLeft() {
        return !mBeamBreakLeft.get();
    }

    boolean hasCageRight() {
        return !mBeamBreakRight.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hang/Left Beam", hasCageLeft());
        SmartDashboard.putBoolean("Hang/Right Beam", hasCageRight());
    }
}
