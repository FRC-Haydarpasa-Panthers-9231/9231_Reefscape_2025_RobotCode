package frc.robot.subsystems.processor_roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_ProcessorRoller extends SubsystemBase {

  private final IO_ProcessorRollerBase io;
  public final ProcessorRollerInputsAutoLogged inputs = new ProcessorRollerInputsAutoLogged();

  public SUB_ProcessorRoller(IO_ProcessorRollerBase io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ProcessorRoller", inputs);
  }

  public void stopMotor() {
    io.stopMotor();
  }

  public void setSpeed(double speed) {
    io.setProcessorRollerSpeed(speed);
  }
}
