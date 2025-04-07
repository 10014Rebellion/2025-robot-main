package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

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

  public enum ledPatterns {
    RED(LEDPattern.solid(ledColor.RED.getColorObj())),
    BLUE(LEDPattern.solid(ledColor.BLUE.getColorObj())),
    GREEN(LEDPattern.solid(ledColor.GREEN.getColorObj())),
    YELLOW(LEDPattern.solid(ledColor.YELLOW.getColorObj()));

    public final LEDPattern pattern;

    private ledPatterns(LEDPattern pPattern) {
      this.pattern = pPattern;
    }

    public LEDPattern getPattern() {
      return this.pattern;
    }
  }

  public enum ledColor {
    RED(60), // RGB: 255, 255, 0
    BLUE(120), // RGB: 0, 255, 0
    GREEN(20), // RGB: 255, 85, 0
    YELLOW(50); // RGB: 255, 213, 0
    // PURPLE(90), // RGB: 128, 255, 0

    public final int color;

    private ledColor(int color) {
      this.color = color;
    }

    public int getHue() {
      return this.color;
    }

    public Color getColorObj() {
      int[] rgbArr = hsvToRgb(this.color, 255, 128);
      return new Color(rgbArr[0], rgbArr[1], rgbArr[2]);
    }
  };

  public static final class RGBLEDColor {
    // These colors are gotten online / through trial and error
    // Please dont touch them it would be a pain to fix ;-;
    public static final int[] red = {255, 255, 0};
    public static final int[] blue = {25, 25, 150};
    public static final int[] orange = {0, 255, 0};
    public static final int[] yellow = {255, 191, 0};
    public static final int[] green = {0, 128, 0};
  }

  // public static final class RGBLEDColor {
  //   // These colors are gotten online / through trial and error
  //   // Please dont touch them it would be a pain to fix ;-;
  //   public static final int[] red = {240, 0, 0};
  //   public static final int[] orange = {255, 45, 0};
  //   public static final int[] yellow = {255, 191, 0};
  //   public static final int[] green = {0, 128, 0};
  //   public static final int[] blue = {25, 25, 150};
  // }

  public static int LEDsPerMeter = 864;

  private static int[] hsvToRgb(int h, int s, int v) {
    // Scale h from [0,180) to [0,360)
    double hue = h * 2.0;
    // Convert s and v from [0,255] to [0,1]
    double sat = s / 255.0;
    double value = v / 255.0;
    double r, g, b;
    // If there is no saturation, the color is a shade of gray
    if (sat == 0) {
      r = g = b = value;
    } else {
      // Convert hue to a sector from 0 to 6
      hue /= 60.0;
      int i = (int) Math.floor(hue);
      double f = hue - i;
      double p = value * (1 - sat);
      double q = value * (1 - sat * f);
      double t = value * (1 - sat * (1 - f));
      switch (i) {
        case 0:
          r = value;
          g = t;
          b = p;
          break;
        case 1:
          r = q;
          g = value;
          b = p;
          break;
        case 2:
          r = p;
          g = value;
          b = t;
          break;
        case 3:
          r = p;
          g = q;
          b = value;
          break;
        case 4:
          r = t;
          g = p;
          b = value;
          break;
        case 5:
        default:
          r = value;
          g = p;
          b = q;
          break;
      }
    }
    // Scale the results back to [0,255] and round
    int R = (int) Math.round(r * 255);
    int G = (int) Math.round(g * 255);
    int B = (int) Math.round(b * 255);
    return new int[] {R, G, B};
  }
}
