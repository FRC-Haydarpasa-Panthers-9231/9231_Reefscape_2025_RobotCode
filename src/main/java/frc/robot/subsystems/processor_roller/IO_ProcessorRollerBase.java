package frc.robot.subsystems.processor_roller;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ProcessorRollerBase {

  @AutoLog
  public static class ProcessorRollerInputs {
    public double processorRollerAppliedVolts = 0.0;
    public double processorRollerCurrentAmps = 0.0;
    public boolean hasAlgea = false;
  }

  void updateInputs(ProcessorRollerInputs inputs);

  /**
   * Motorun hızını ayarlar.
   *
   * @param speed Çalıştırılmak istenen hız degeri. [-1,1] arasında deger alır.
   */
  public void setProcessorRollerSpeed(double speed);

  /** Motoru durdurur */
  public void stopMotor();

  public boolean hasAlgae();

  public void setHasAlgaeOverride(boolean passedHasGamePiece);

  public void algaeToggle();

  public double getAlgaeIntakeVoltage();

  public void setAlgaeIntakeVoltage(double voltage);
}
