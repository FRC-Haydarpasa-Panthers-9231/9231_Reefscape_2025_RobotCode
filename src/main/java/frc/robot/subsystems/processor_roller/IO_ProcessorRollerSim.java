package frc.robot.subsystems.processor_roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class IO_ProcessorRollerSim implements IO_ProcessorRollerBase {

  private DCMotorSim processorRoller1Motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.00032, 1),
          DCMotor.getNeoVortex(1));

  private double processorRoller1MotorAppliedVolts = 0.0;
  private boolean hasAlgaeOverride = false;

  public IO_ProcessorRollerSim() {}

  @Override
  public void updateInputs(ProcessorRollerInputs inputs) {
    inputs.processorRollerAppliedVolts = processorRoller1MotorAppliedVolts;
    inputs.processorRollerCurrentAmps = processorRoller1Motor.getCurrentDrawAmps();
    inputs.hasAlgea = true;
    Logger.recordOutput("ProcessorRoller/hasAlgea", hasAlgae());
  }

  @Override
  public void setProcessorRollerSpeed(double speed) {
    processorRoller1MotorAppliedVolts = MathUtil.clamp(12 * speed, -12, 12);
    processorRoller1Motor.setInputVoltage(processorRoller1MotorAppliedVolts);
  }

  @Override
  public void stopMotor() {
    processorRoller1MotorAppliedVolts = 0;
    processorRoller1Motor.setInputVoltage(processorRoller1MotorAppliedVolts);
  }

  public boolean hasAlgae() {
    // Motorun mevcut çektigi akım degerini al
    double intakeCurrent = processorRoller1Motor.getCurrentDrawAmps();

    // Motorun uyguladıgı çıkış voltajını al
    double appliedVoltage = processorRoller1MotorAppliedVolts;

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
    return processorRoller1MotorAppliedVolts;
  }

  public void setProcessorRollerVoltage(double voltage) {
    processorRoller1MotorAppliedVolts = MathUtil.clamp(voltage, -12, 12);
    processorRoller1Motor.setInputVoltage(processorRoller1MotorAppliedVolts);
  }
}
