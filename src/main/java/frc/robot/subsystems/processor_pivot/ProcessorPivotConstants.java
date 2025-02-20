package frc.robot.subsystems.processor_pivot;

public class ProcessorPivotConstants {
  // TODO: motor IDsini ve channel idsini ata
  public static final int kProcessorPivotMotorID = 1;
  public static final int kProcessorPivotEncoderChannel = 14;

  // TODO İNVERTLENİP İNVERTLENMEYECEgİNE KARAR VER.
  public static final boolean kIsInverted = false;

  // MAX VE MIN PID çıktısı
  public static final double kProcessorPivotMinOutput = 0;
  public static final double kProcessorPivotMaxOutput = 2;

  // TODO: Limitlerini tespit et
  public static final double kProcessorPivotMinAngle = 0;
  public static final double kProcessorPivotMaxAngle = 50;

  // TODO: REDUCTİON VARSA KONTROL ET
  public static final double kGearing = 1;

  // Simulasyon için
  public static final double kArmLength = 1;
  public static final double kMass = 1;

  // TODO: PID Degerlerini bul
  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final boolean kBrake = true;

  public static final double CLEANING_REEF_L2_PIVOT_POSITION = 0.0;
  public static final double CLEANING_REEF_L3_PIVOT_POSITION = 0.0;
  public static final double INTAKING_ALGEA_GROUND_POSITION = 0.0;
  public static final double ARM_ZERO_POSITION = 0.0;
}
