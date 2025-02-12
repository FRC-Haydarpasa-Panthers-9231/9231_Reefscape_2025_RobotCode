package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ElevatorBase {

  @AutoLog
  public static class ElevatorInputs {
    public double[] elevatorAppliedVolts = new double[] {};
    public double[] elevatorCurrentAmps = new double[] {};
    public double[] elevatorTempCelcius = new double[] {};
    public double elevatorPositionRad = 0.0;
    public double elevatorVelocityRadPerSec = 0.0;
  }

  void updateInputs(ElevatorInputs inputs);

  /**
   * Asansör motorlarının voltunu ayarlar
   *
   * @param volts Çalıştırılacak volt değeri. [-12,12] volt değerlerinde çalışır.
   */
  public void setElevatorVoltage(double volts);

  /**
   * Asansörün hızını ayarlar
   *
   * @param speed Çalıştırılmak istenen hız değeri. [-1,1] arasında değer alır.
   */
  public void setElevatorSpeed(double speed);

  /**
   * PID kontrolünde kullanılacak sabitler
   *
   * @param kG Yerçekimi telafisi için kullanılan sabit (Gravity compensation). Sistemin kütlesinden
   *     dolayı oluşan yerçekimi etkisini telafi etmek için kullanılır.
   * @param kS Statik sürtünme sabiti (Static friction). Motor harekete geçmek için aşması gereken
   *     minimum kuvveti temsil eder. Robotun durduğu pozisyondan harekete geçebilmesi için
   *     kullanılır.
   * @param kV Hız sabiti (Velocity constant). Motorun belirli bir hızda dönmesini sağlamak için
   *     gereken kuvveti temsil eder. Genellikle birim hız başına volt olarak ölçülür.
   * @param kA İvme sabiti (Acceleration constant). İvme sırasında motorun çıkış voltajını artırmak
   *     veya azaltmak için kullanılır. Robotun daha kararlı ve pürüzsüz hareket etmesine yardımcı
   *     olur.
   * @param kP Orantılı kazanç (Proportional gain). Hata ile doğru orantılı bir düzeltme uygular.
   *     Hata arttıkça, daha büyük bir düzeltme uygulanır.
   * @param kI İntegral kazanç (Integral gain). Zaman içinde biriken hatayı düzeltmek için
   *     kullanılır. Sistemde sürekli bir hata oluşuyorsa, bu hatayı sıfırlamak için kullanılır.
   * @param kD Türevsel kazanç (Derivative gain). Hata değişim hızına göre bir düzeltme uygular.
   *     Sistemin aşırı tepki vermesini ve osilasyon yapmasını önlemek için kullanılır.
   */
  public void setSlot0(double kG, double kS, double kV, double kA, double kP, double kI, double kD);

  /** Asansör motorlarını durdurur. */
  public void stopMotor();

  /**
   * Asansörü belli bir pozisyona götürür.
   *
   * @param positionRad başlangıç konumundan motorun radyan cinsinden sabitlenmek istendiği konum.
   */
  public void runPosition(double positionRad);
}
