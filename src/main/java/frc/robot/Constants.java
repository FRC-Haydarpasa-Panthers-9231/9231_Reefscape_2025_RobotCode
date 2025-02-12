package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true;

  public static class Elevator {
    public static final int kElevatorMotor1Port = 9;
    public static final int kElevatorMotor2Port = 10;

    public static final double kElevatorGearing = 25;
    public static final double kCarriageMass = 18;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(0.62);
    public static final double kMinElevatorHeightMeters = 0;
    public static final double kMaxElevatorHeightMeters = 1.6075;
    public static final double kDefaultSetpoint = 0;

    public static final int kElevatorTeeth = 17;
    public static final double kElevatorPitch = 6.35;
  }

  public static class ElevatorRoller {
    public static final int kElevatorRollerMotor1Port = 11;
    public static final int kElevatorRollerMotor2Port = 12;
  }

  public static class ProcessorPivot {
    public static final int kProcessorPivotMotorPort = 13;
    public static final int kProcessorPivotEncoderChannel = 14;
    public static final double kProcessorPivotMinOutput = 0;
    public static final double kProcessorPivotMaxOutput = 2;

    public static final double kProcessorPivotMinAngleRad = 0;
    public static final double kProcessorPivotMaxAngleRad = 50;

    public static final double kGearing = 1;
    public static final double kArmLength = 1;
    public static final double kMass = 1;

    // TODO: BUNU BUL
    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
  }

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
