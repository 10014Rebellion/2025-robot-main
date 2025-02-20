package frc.robot.subsystems.LEDs;

public class LEDConstants {
  public static final class HSVLEDColor {
    public static final int red = 90;
    public static final int orange = 100;
    public static final int yellow = 70;
    public static final int green = 30;
    public static final int blue = 0;
    public static final int indigo = 120;
    public static final int purple = 140;
  }

  public static final class RGBLEDColor {
    // These colors are gotten online / through trial and error
    // Please dont touch them it would be a pain to fix ;-;
    public static final int[] red = {240, 0, 0};
    public static final int[] orange = {255, 45, 0};
    public static final int[] yellow = {255, 191, 0};
    public static final int[] green = {0, 128, 0};
    public static final int[] blue = {25, 25, 150};
  }

  public static int LEDsPerMeter = 864;
}
