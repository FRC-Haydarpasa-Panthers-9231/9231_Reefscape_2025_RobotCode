package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  // set to true in order to change all Tunable values via AdvantageScope
  public static final boolean TUNING_MODE = false;
  public static final boolean DEMO_MODE = false;
  public static final boolean DEBUG = false;
  public static final boolean AT_COMPETITION = false;
  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final int kElevatorLimitSwitchPort = 2;
  public static final int kBeamBreakPort = 3;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kPracticeControllerPort = 2;
    public static final double LEFT_X_DEADBAND = 0.12;
    public static final double LEFT_Y_DEADBAND = 0.12;
    public static final double RIGHT_X_DEADBAND = 0.12;
  }

  private static final RobotType ROBOT = RobotType.ROBOT_COMPETITION;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError);

  public enum RobotType {
    ROBOT_DEFAULT,
    ROBOT_SIMBOT,
    ROBOT_PRACTICE,
    ROBOT_COMPETITION,
    ROBOT_PRACTICE_BOARD,
    ROBOT_VISION_TEST_PLATFORM
  }

  // FIXME: update for various robots
  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_COMPETITION;
      } else {
        return ROBOT;
      }
    } else {
      return ROBOT;
    }
  }

  // FIXME: update for various robots
  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_DEFAULT, ROBOT_PRACTICE, ROBOT_PRACTICE_BOARD, ROBOT_COMPETITION:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT, ROBOT_VISION_TEST_PLATFORM:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }
}
