package frc.robot.subsystems.processor_pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ProcessorPivotBase {

  @AutoLog
  public static class ProcessorPivotInputs {
    public double processorPivotAppliedVolts = 0.0;
    public double processorPivotCurrentAmps = 0.0;
    public double elevatorPositionRad = 0.0;
  }

  void updateInputs(ProcessorPivotInputs inputs);

  public double getProcessorPivotPosition();

  /** Motoru durdurur. */
  public void stopMotor();

  /**
   * Motorların hızını ayarlar.
   *
   * @param speed Çalıştırılmak istenen hız değeri. Değer 0 ile 1 arasında
   */
  public void setSpeed(double speed);

  /**
   * Kolun pozisyonunu ayarlar.
   *
   * @param setPoint Kolun sabit durması istenilen pozisyon.
   */
  public void setPosition(double setPoint);

  /**
   * PID değerlerini değiştirmemizi sağlar. Tuning mod açıkken kullanılır.
   *
   * @param kP Orantılı kazanç (Proportional gain). Hata ile doğru orantılı bir düzeltme uygular.
   *     Hata arttıkça, daha büyük bir düzeltme uygulanır.
   * @param kI İntegral kazanç (Integral gain). Zaman içinde biriken hatayı düzeltmek için
   *     kullanılır. Sistemde sürekli bir hata oluşuyorsa, bu hatayı sıfırlamak için kullanılır.
   * @param kD Türevsel kazanç (Derivative gain). Hata değişim hızına göre bir düzeltme uygular.
   *     Sistemin aşırı tepki vermesini ve osilasyon yapmasını önlemek için kullanılır.
   */
  public void setPID(double kP, double kI, double kD);
}
