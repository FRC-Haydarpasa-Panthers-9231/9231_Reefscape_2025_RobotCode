package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final double loopPeriodSecs = 0.02;
    public static final boolean tuningMode = true;
    public static final boolean debug = true;

    public static final boolean DEMO_MODE = false;
    public static final boolean kIsCompetition = false;

    public static final int kBeamBreakPort = 1;


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
