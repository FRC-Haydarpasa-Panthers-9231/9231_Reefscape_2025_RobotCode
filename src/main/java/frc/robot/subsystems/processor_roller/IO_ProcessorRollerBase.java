package frc.robot.subsystems.processor_roller;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ProcessorRollerBase {

  @AutoLog
  public static class ProcessorRollerInputs {
    public double processorRollerAppliedVolts = 0.0;
    public double processorRollerCurrentAmps = 0.0;
  }

  void updateInputs(ProcessorRollerInputs inputs);

  /**
   * Motorun hızını ayarlar.
   *
   * @param speed Çalıştırılmak istenen hız değeri. [-1,1] arasında değer alır.
   */
  public void setProcessorRollerSpeed(double speed);

  /** Motoru durdurur */
  public void stopMotor();
}
