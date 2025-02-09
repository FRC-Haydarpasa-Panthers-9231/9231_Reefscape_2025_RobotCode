package frc.robot.util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Builder {

  /**
   * Yeni bir spark max oluşturur
   *
   * @param ID Spark max ID'si
   * @param invert Dönüş yönünü tersine çevirmek için true verin
   * @param setBrakeMode Fren modu için true salma modu için false verin
   */
  public static SparkMax initNeo(int ID, boolean invert, boolean setBrakeMode) {
    final SparkMax newSpark = new SparkMax(ID, MotorType.kBrushless);

    newSpark.configure(
        new SparkMaxConfig()
            .idleMode(setBrakeMode ? IdleMode.kBrake : IdleMode.kBrake)
            .smartCurrentLimit(55)
            .inverted(invert),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    return newSpark;
  }
}
