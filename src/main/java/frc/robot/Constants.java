package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;

  public static class Elevator {
    public static final double kElevatorGearing = 5;
    public static final double kCarriageMass = 3;
    public static final double kElevatorDrumRadius = 0.3;
    public static final double kMinElevatorHeightMeters = 5;
    public static final double kMaxElevatorHeightMeters = 10;
    public static final double kDefaultSetpoint = 0;
  }

  // Yazılımsal limit için maksimum ve minimum pozisyon değerleri
  public static final double ELEVATOR_MAX_POSITION_RAD = 0; // Örnek: 90 derece
  public static final double ELEVATOR_MIN_POSITION_RAD = 100; // Örnek: 0 derece

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kPracticeControllerPort = 2;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
