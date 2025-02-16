package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ElevatorBase {

  @AutoLog
  public static class ElevatorInputs {
    double voltageSuppliedLead = 0.0;
    double voltageSuppliedFollower = 0.0;

    double statorCurrentAmpsLead = 0.0;
    double statorCurrentAmpsFollower = 0.0;

    double supplyCurrentAmpsLead = 0.0;
    double supplyCurrentAmpsFollower = 0.0;

    double closedLoopError = 0.0;

    double closedLoopReference = 0.0;

    double positionMeters = 0.0;

    double positionRotations = 0.0;

    double leadTempCelsius = 0.0;
    double followerTempCelsius = 0.0;
  }

  void updateInputs(ElevatorInputs inputs);

  /**
   * Asansör motorlarının voltunu ayarlar
   *
   * @param volts Çalıştırılacak volt değeri. [-12,12] volt değerlerinde çalışır.
   */
  public void setElevatorVoltage(double volts);

  public default void zeroPosition() {}

  /**
   * Asansörün hızını ayarlar
   *
   * @param speed Çalıştırılmak istenen hız değeri. [-1,1] arasında değer alır.
   */
  public void setElevatorSpeed(double speed);

  /** Asansör motorlarını durdurur. */
  public void stopMotor();

  public void runPositionRads(double rads);

  /**
   * Asansörü belli bir pozisyona götürür.
   *
   * @param positionRad başlangıç konumundan motorun radyan cinsinden sabitlenmek istendiği konum.
   */
  public void runPositionMeters(double positionMeters);
}
