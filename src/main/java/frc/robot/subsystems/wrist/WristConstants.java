package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class WristConstants {

    public static int kMotorID = 42;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;

    public static boolean kMotorInverted = true;
    public static int kSmartCurrentLimit = 60;
    public static int kSecondaryCurrentLimit = 80; //TODO: FIND
    public static double kForwardSoftLimit = 90;
    public static double kReverseSoftLimit = -89;
    public static double kPositionConversionFactor = 360.0;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static int kEncoderPort = 1;
    public static double kEncoderOffsetDeg = -162.95;
    public static boolean kEncoderInverted = true;

    // NO PIECE
    public static double k0P = 0.22;
    public static double k0I = 0.0;
    public static double k0D = 0.01;
    public static double k0MaxAcceleration = 1000; // was 500. k0inda slow
    public static double k0MaxVelocity = 700;
    public static double k0S = 0.02;
    public static double k0G = 0.4;
    public static double k0V = 0.003;
    public static double k0A = 0.0;
    public static double k0Tolerance = 2.3;
  
    // CORAL
    public static double k1P = 0.22;
    public static double k1I = 0.0;
    public static double k1D = 0.01;
    public static double k1MaxAcceleration = 900; // was 500. k1inda slow
    public static double k1MaxVelocity = 350;
    public static double k1S = 0.02;
    public static double k1G = 0.4;
    public static double k1V = 0.003;
    public static double k1A = 0.0;
    public static double k1Tolerance = 2.3;
  
    // ALGAE THROW
    public static double k2P = 0.22;
    public static double k2I = 0.0;
    public static double k2D = 0.01;
    public static double k2MaxAcceleration = 1100; // was 500. k2inda slow
    public static double k2MaxVelocity = 600;
    public static double k2S = 0.02;
    public static double k2G = 0.4;
    public static double k2V = 0.003;
    public static double k2A = 0.0;
    public static double k2Tolerance = 1.7;

    public static double kWristDownLimit = -30;
    public static double throwAlgaePos = 50;


    public static WristHardware wristHardware = new WristHardware(kMotorID, kMotorType);

    public static EncoderHardware encoderHardware = new EncoderHardware(kEncoderPort, kEncoderOffsetDeg, kEncoderInverted);

    public static WristConfiguration motorConfiguration = new WristConfiguration(
        kSmartCurrentLimit, 
        kSecondaryCurrentLimit, 
        kIdleMode, 
        kMotorInverted, 
        kForwardSoftLimit, 
        kReverseSoftLimit, 
        kPositionConversionFactor, 
        kVelocityConversionFactor);

    ControllerConfiguration controller0 = new ControllerConfiguration(
        k0P, 
        k0I, 
        k0D, 
        k0S, 
        k0V, 
        k0A, 
        k0G, 
        k0MaxVelocity, 
        k0MaxAcceleration, 
        k0Tolerance);


    ControllerConfiguration controller1 = new ControllerConfiguration(
        k1P, 
        k1I, 
        k1D, 
        k1S, 
        k1V, 
        k1A, 
        k1G, 
        k1MaxVelocity, 
        k1MaxAcceleration, 
        k1Tolerance);

    ControllerConfiguration controller2 = new ControllerConfiguration(
        k2P, 
        k2I, 
        k2D, 
        k2S, 
        k2V, 
        k2A, 
        k2G, 
        k2MaxVelocity, 
        k2MaxAcceleration, 
        k2Tolerance);


    public record WristHardware(
        int kMotorPort,
        MotorType kMotorType
    ){}

    public record EncoderHardware(
        int kEncoderPort,
        double kEncoderOffset,
        boolean kEncoderInverted
    ){}

    public record WristConfiguration(
        int kSmartLimit,
        int kSecondaryLimit,
        IdleMode kIdleMode,
        boolean kInverted,
        double kForwardSoftLimit,
        double kReverseSoftLimit,
        double kPositionConversionFactor,
        double kVelocityConversionFactor
    ){}


    public record ControllerConfiguration(
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kG,
        double kA,
        double kMaxVelo,
        double kMaxAccel,
        double kTolerance
    ){}


    public enum Setpoints {
        BOTTOM(0),
        INTAKE(-87),
        HPINTAKE(0),
        L1(-30),
        L2(46 + 6),
        L3(46 + 6),
        L4(65),
    
        SCORE(9),
        AUTON_L4_SCORE(2),
        L2SCORE(9),
        BARGE(71),
        L2ALGAE(-28),
        L3ALGAE(25),
        HOLD_ALGAE(0),
        THROW_ALGAE(90),
        CLIMB(-15),
        GROUNDALGAE(-31),
        STOWED(90);
    
        public final double setpoint;
    
        private Setpoints(double setpoint) {
          this.setpoint = setpoint;
        }
    
        public double getPos() {
          return this.setpoint;
        }
      };

}
