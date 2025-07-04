package frc.robot.util.misc;

public class MiscUtils {
  public static boolean isValueInRange(double value, double minValue, double maxValue) {
    return value >= minValue && value <= maxValue;
  }
}
