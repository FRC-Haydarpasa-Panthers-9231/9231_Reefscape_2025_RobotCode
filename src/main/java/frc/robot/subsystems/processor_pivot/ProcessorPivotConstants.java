package frc.robot.subsystems.processor_pivot;

public class ProcessorPivotConstants {
  // TODO: motor IDsini ve channel idsini ata
  public static final int kProcessorPivotMotorID = 1;
  public static final int kProcessorPivotEncoderChannel = 14;
  public static final boolean kIsInverted = false;

  // MAX VE MIN PID çıktısı
  public static final double kProcessorPivotMinOutput = 0;
  public static final double kProcessorPivotMaxOutput = 2;

  // TODO: Limitlerini tespit et
  public static final double kProcessorPivotMinAngle = 0;
  public static final double kProcessorPivotMaxAngle = 50;

  // Reduction
  public static final double kGearing = 1;

  // Simulasyon için
  public static final double kArmLength = 1;
  public static final double kMass = 1;

  // TODO: PID Değerlerini bul
  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  // Motorun invertli olup olmadığı
  public static final boolean kBrake = false;
}
