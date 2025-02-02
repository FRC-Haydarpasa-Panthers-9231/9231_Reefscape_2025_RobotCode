package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Builder {

  /**
   * Yeni bir kraken motoru oluşturur ve döndürür
   *
   * @param ID Kraken motorun ID'si
   * @param invert Motorun dönüş yönünü tersine çevirir
   * @param setBrakeMode Fren moduna almak için true değerini verin. Motorun kendini salması için
   *     false değerini verin.
   * @param currentLimitAmps Voltaj limiti. Motoru yanmaktan kurtarabilir. 40 Ya da 70-80 önerilir.
   */
  public static TalonFX initKraken(int ID, boolean invert, boolean brake, int currentLimitAmps) {
    final TalonFX newKraken = new TalonFX(ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    newKraken.getConfigurator().apply(config);
    return newKraken;
  }

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

  /**
   * Sets the provided Spark Max as a follower of the specified master Spark Max.
   *
   * @param master The master Spark Max
   * @param follower The Spark Max to set as a follower
   * @return The modified follower Spark Max
   */
  public static SparkMax setNeoFollower(SparkMax master, SparkMax follower) {

    return follower;
  }
}
