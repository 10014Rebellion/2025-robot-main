package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Beambreak extends SubsystemBase {
  private final DigitalInput mBeamBreakHangRight;
  private final DigitalInput mBeamBreakHangLeft;

  public Beambreak() {
    mBeamBreakHangRight = new DigitalInput(3);
    mBeamBreakHangLeft = new DigitalInput(4);
  }

  public boolean hasCageLeft() {
    return !mBeamBreakHangRight.get();
  }

  public boolean hasCageRight() {
    return !mBeamBreakHangLeft.get();
  }

  public boolean hasCage() {
    return hasCageLeft() && hasCageRight();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Hang Beam", hasCageLeft());
    SmartDashboard.putBoolean("Right Hang Beam", hasCageRight());
  }
}
