package frc.robot.subsystems.processor_roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SUB_ProcessorRoller extends SubsystemBase {

  private final IO_ProcessorRollerBase io;
  public final ProcessorRollerInputsAutoLogged inputs = new ProcessorRollerInputsAutoLogged();
  private static final LoggedTunableNumber processorRollerVoltage =
      new LoggedTunableNumber("ProcessorRoller/Processor Roller Voltage", 2);

  private boolean hasAlgeaOverride = false;

  public SUB_ProcessorRoller(IO_ProcessorRollerBase io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ProcessorRoller", inputs);
    Logger.recordOutput("HasAlgeaOverride", hasAlgeaOverride);
  }

  public void stopMotor() {
    io.stopMotor();
  }

  public void setSpeed(double speed) {
    if (speed > 0) {
      hasAlgeaOverride = false;
    }
    io.setProcessorRollerSpeed(speed);
  }

  public boolean hasAlgae() {
    if (hasAlgeaOverride) {
      return true;
    }
    return io.hasAlgae();
  }

  public void setProcessorRollerVoltage(double voltage) {
    io.setProcessorRollerVoltage(voltage);
  }

  public void setProcessorRollerDebugVoltage(boolean directionPositive) {
    int directionVal = directionPositive ? 1 : -1;
    io.setProcessorRollerVoltage(processorRollerVoltage.getAsDouble() * directionVal);
  }

  public double getAlgaeIntakeVoltage() {
    return io.getAlgaeIntakeVoltage();
  }

  public void algaeToggle() {
    io.algaeToggle();
  }

  public void setHasAlgaeOverride(boolean passedHasGamePiece) {
    io.setHasAlgaeOverride(passedHasGamePiece);
  }
}
