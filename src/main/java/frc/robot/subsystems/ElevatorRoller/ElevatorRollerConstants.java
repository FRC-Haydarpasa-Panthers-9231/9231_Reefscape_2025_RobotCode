package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;

public class ElevatorRollerConstants {

    // TODO: PORTLARI KONTROL ET VE DOgRU PORTLARI ATA
    public static final int kElevatorRollerMotor1Port = 13;
    public static final int kElevatorRollerMotor2Port = 14;
    public static final String kSubsystemName = "Elevator Roller";

    // TODO SCORİNG SPEEDLERİ ÖLÇ
    public static final double kL1ScoringSpeed = 0.6;
    public static final double kL2ScoringSpeed = 0.6;
    public static final double kL3ScoringSpeed = 0.6;
    public static final double kL4ScoringSpeed = 0.6;

    // TODO: SCORE TİME OPTİMİZE OLABİLİRSE OPTİMİZE ET
    public static final Time kCoralScoreTime = Units.Seconds.of(2);
    public static final Time kFeedTimeout = Units.Seconds.of(4);
    public static final Time kAfterSensorWaitTime = Units.Seconds.of(0.15);


    public static final double kIntakingSpeed = 0.5;
    public static final double kAfterSensorIntakingSpeed = 0.2;

}
