package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
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

    double velocityLead = 0.0;
    double velocityFollower = 0.0;

    double closedLoopError = 0.0;

    double closedLoopReference = 0.0;

    Distance positionMeters = Meters.of(0);

    double positionRotations = 0.0;
    double positionRads = 0.0;

    double tempCelciusLead = 0.0;
    double tempCelciusFollower = 0.0;
  }

  void updateInputs(ElevatorInputs inputs);

  /**
   * Asansör motorlarının voltunu ayarlar
   *
   * @param volts Çalıştırılacak volt degeri. [-12,12] volt degerlerinde çalışır.
   */
  public void setElevatorVoltage(Voltage volts);

  public Distance getElevatorPosition();

  public void setSensorPosition(Distance setpoint);

  /**
   * Asansörün hızını ayarlar
   *
   * @param speed Çalıştırılmak istenen hız degeri. [-1,1] arasında deger alır.
   */
  public void setElevatorSpeed(double speed);

  /** Asansör motorlarını durdurur. */
  public void stopMotor();

  public void runPositionRads(double rads);

  public AngularVelocity getRotorVelocity();

  public void setNeutral();

  public void setCoastMode(Boolean coastMode);

  public boolean isRotorVelocityZero();

  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable);

  public void setPosition(Distance height);
}
