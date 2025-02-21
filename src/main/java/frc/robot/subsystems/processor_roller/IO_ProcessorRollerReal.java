package frc.robot.subsystems.processor_roller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class IO_ProcessorRollerReal implements IO_ProcessorRollerBase {

  private final Alert configAlert =
      new Alert("Processor Roller için config ayarlanırken bir hata oluştu.", AlertType.kError);

  private SparkMax processorRoller;

  private boolean hasAlgaeOverride = false;

  public IO_ProcessorRollerReal() {
    processorRoller =
        new SparkMax(ProcessorRollerConstants.kProcessorRollerPort, MotorType.kBrushless);

    SparkUtil.tryUntilOk(
        processorRoller,
        5,
        () ->
            processorRoller.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(50),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        configAlert);
    FaultReporter.getInstance()
        .registerHardware(
            ProcessorRollerConstants.kSubsystemName, "Processor Roller Motor ", processorRoller);
  }

  @Override
  public void updateInputs(ProcessorRollerInputs inputs) {
    inputs.processorRollerAppliedVolts =
        processorRoller.getAppliedOutput() * processorRoller.getBusVoltage();
    inputs.processorRollerCurrentAmps = processorRoller.getOutputCurrent();
    inputs.hasAlgea = hasAlgae();
    Logger.recordOutput("ProcessorRoller/hasAlgeaOverride", hasAlgaeOverride);
  }

  @Override
  public void setProcessorRollerSpeed(double speed) {
    processorRoller.set(speed);
  }

  @Override
  public void stopMotor() {
    processorRoller.stopMotor();
  }

  public boolean hasAlgae() {
    // Motorun mevcut çektigi akım degerini alıyoruz
    double intakeCurrent = processorRoller.getOutputCurrent();

    // Motorun uyguladıgı çıkış voltajını alıyoruz
    double appliedVoltage = processorRoller.getAppliedOutput() * processorRoller.getBusVoltage();

    // Eşik degerler (constAlgaeIntake'ten alınan sabitler)
    double processorRollerHasGamePieceCurrent =
        ProcessorRollerConstants.kAlgeaIntakeHasGamePieceCurrent;

    double intakeHasGamePieceVoltage =
        ProcessorRollerConstants.kAlgeaProcessorRollerHasGamePieceVoltage;

    // Manuel override kontrolü
    if (hasAlgaeOverride) {
      return hasAlgaeOverride;
    }

    // Algae algılama koşulları:
    // 1. Akım belirli bir eşik degerden büyükse
    // 2. Motor voltajı belirli bir eşik degerden küçükse (yük altındaysa yavaşlama göstergesi)
    if ((intakeCurrent >= processorRollerHasGamePieceCurrent)
        && (appliedVoltage <= intakeHasGamePieceVoltage)) {
      return true;
    } else {
      return false;
    }
  }

  public void setHasAlgaeOverride(boolean passedHasGamePiece) {
    hasAlgaeOverride = passedHasGamePiece;
  }

  public void algaeToggle() {
    this.hasAlgaeOverride = !hasAlgaeOverride;
  }

  public double getAlgaeIntakeVoltage() {
    return processorRoller.getAppliedOutput() * processorRoller.getBusVoltage();
  }

  public void setProcessorRollerVoltage(double voltage) {
    processorRoller.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }
}
