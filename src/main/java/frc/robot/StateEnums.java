package frc.robot;

public class StateEnums {

  public static class Manipulator {
    public enum Level {
      L1,
      L2,
      L3,
      L4
    }

    public enum Side {
      LEFT,
      CENTER,
      RIGHT
    }
  }

  public static class Intake {
    public enum Current {
      MANUAL,
      MOVING_TO_SETPOINT,
      DEPLOYED,
      RETRACTED
    }

    public enum Setpoint {
      DEPLOYED,
      RETRACTED,
      NONE
    }
  }

  public static class Elevator {
    public enum Current {
      L1,
      L2,
      L3,
      L4,
      BARGE,
      MANUAL,
      RETRACTED
    }

    public enum Setpoint {
      L1,
      L2,
      L3,
      L4,
      BARGE,
      RETRACTED,
      NONE
    }
  }

  public static class Drive {
    public enum Current {
      MANUAL,
      MOVING_TO_SETPOINT,
      AT_SETPOINT,
    }

    public enum Setpoint {
      REEF_POSE,
      BARGE_POSE,
      NONE
    }
  }

  public static class Claw {
    public enum Current {
      L1,
      L2,
      L3,
      L4,
      BARGE,
      INDEX,
      MANUAL
    }

    public enum Setpoint {
      L1,
      L2,
      L3,
      L4,
      BARGE,
      INDEX,
      NONE
    }
  }
}
