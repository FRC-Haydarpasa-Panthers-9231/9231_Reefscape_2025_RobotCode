package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;

public class ElevatorRollerConstants {

  // TODO: PORTLARI KONTROL ET VE DOgRU PORTLARI ATA
  public static final int kElevatorRollerMotor1Port = 1;
  public static final int kElevatorRollerMotor2Port = 1;

  // TODO SCORİNG SPEEDLERİ ÖLÇ
  public static final double kL1ScoringSpeed = 1;
  public static final double kL2ScoringSpeed = 1;
  public static final double kL3ScoringSpeed = 1;
  public static final double kL4ScoringSpeed = 1;

  // TODO: SCORE TİME OPTİMİZE OLABİLİRSE OPTİMİZE ET
  public static final Time kCoralScoreTime = Units.Seconds.of(0.2);
  public static final Time kFeedTimeout = Units.Seconds.of(2);
}
