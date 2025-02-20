package frc.robot.subsystems.processor_pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ProcessorPivotBase {

  @AutoLog
  public static class ProcessorPivotInputs {
    public double processorPivotAppliedVolts = 0.0;
    public double processorPivotCurrentAmps = 0.0;
    public double processorPivotTempCelcius = 0.0;
    public double processorPivotPositionRads = 0.0;
  }

  void updateInputs(ProcessorPivotInputs inputs);

  public double getProcessorPivotPosition();

  /** Motoru durdurur. */
  public void stopMotor();

  /**
   * Motorların hızını ayarlar.
   *
   * @param speed Çalıştırılmak istenen hız degeri. Deger 0 ile 1 arasında
   */
  public void setSpeed(double speed);

  /**
   * Kolun pozisyonunu ayarlar.
   *
   * @param setPoint Kolun sabit durması istenilen pozisyon.
   */
  public void setPosition(double setPoint);

  /**
   * PID degerlerini degiştirmemizi saglar. Tuning mod açıkken kullanılır.
   *
   * @param kP Orantılı kazanç (Proportional gain). Hata ile dogru orantılı bir düzeltme uygular.
   *     Hata arttıkça, daha büyük bir düzeltme uygulanır.
   * @param kI İntegral kazanç (Integral gain). Zaman içinde biriken hatayı düzeltmek için
   *     kullanılır. Sistemde sürekli bir hata oluşuyorsa, bu hatayı sıfırlamak için kullanılır.
   * @param kD Türevsel kazanç (Derivative gain). Hata degişim hızına göre bir düzeltme uygular.
   *     Sistemin aşırı tepki vermesini ve osilasyon yapmasını önlemek için kullanılır.
   */
  public void setPID(double kP, double kI, double kD);
}
