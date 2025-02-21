package frc.robot.subsystems.processor_roller;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class IO_ProcessorRollerSim implements IO_ProcessorRollerBase {

  private final Alert configAlert =
      new Alert("Processor Roller için config ayarlanırken bir hata oluştu.", AlertType.kError);

  DCMotor maxGearbox = DCMotor.getNEO(1);

  SparkMax processorRoller =
      new SparkMax(ProcessorRollerConstants.kProcessorRollerPort, MotorType.kBrushless);

  SparkMaxConfig config = new SparkMaxConfig();

  SparkMaxSim processorRollerSim = new SparkMaxSim(processorRoller, maxGearbox);

  private boolean hasAlgaeOverride = false;

  public IO_ProcessorRollerSim() {
    SparkUtil.tryUntilOk(
        processorRoller,
        5,
        () ->
            processorRoller.configure(
                config.idleMode(IdleMode.kBrake).smartCurrentLimit(50),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        configAlert);
    FaultReporter.getInstance()
        .registerHardware(
            ProcessorRollerConstants.kSubsystemName, "Processor Roller Motor", processorRoller);
  }

  @Override
  public void updateInputs(ProcessorRollerInputs inputs) {

    inputs.processorRollerAppliedVolts =
        processorRollerSim.getAppliedOutput() * processorRollerSim.getBusVoltage();
    inputs.processorRollerCurrentAmps = processorRollerSim.getMotorCurrent();

    inputs.hasAlgea = true;
    Logger.recordOutput("ProcessorRoller/hasAlgea", hasAlgae());

    FaultReporter.getInstance()
        .registerHardware(
            ProcessorRollerConstants.kSubsystemName, "Processor Roller Motor ", processorRoller);
  }

  @Override
  public void setProcessorRollerSpeed(double speed) {
    processorRollerSim.setAppliedOutput(speed);
  }

  @Override
  public void stopMotor() {
    processorRollerSim.setAppliedOutput(0);
  }

  public boolean hasAlgae() {
    // Motorun mevcut çektigi akım degerini al
    double intakeCurrent = processorRollerSim.getMotorCurrent();

    // Motorun uyguladıgı çıkış voltajını al
    double appliedVoltage =
        processorRollerSim.getAppliedOutput() * processorRollerSim.getBusVoltage();
    ;

    // Eşik degerler
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
    // 2. Motor voltajı belirli bir eşik degerden küçükse
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
    return processorRollerSim.getAppliedOutput() * processorRollerSim.getBusVoltage();
  }

  public void setProcessorRollerVoltage(double voltage) {
    processorRollerSim.setBusVoltage(voltage);
  }
}
