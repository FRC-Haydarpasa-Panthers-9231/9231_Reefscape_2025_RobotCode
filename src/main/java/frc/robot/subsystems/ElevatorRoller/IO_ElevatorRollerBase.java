package frc.robot.subsystems.ElevatorRoller;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ElevatorRollerBase {

  @AutoLog
  public static class ElevatorRollerInputs {
    public double elevatorRoller1AppliedVolts = 0.0;
    public double elevatorRoller1CurrentAmps = 0.0;
    public double elevatorRoller1TempCelsius = 0.0;

    public double elevatorRoller2AppliedVolts = 0.0;
    public double elevatorRoller2CurrentAmps = 0.0;
    public double elevatorRoller2TempCelsius = 0.0;
  }

  void updateInputs(ElevatorRollerInputs inputs);

  /**
   * @param speed Çalıştırılmak istenen hız
   */
  public void setElevatorRollerSpeed(double speed);

  /** Motorları Durdurur */
  public void stopMotors();

  public boolean hasCoral();
}
