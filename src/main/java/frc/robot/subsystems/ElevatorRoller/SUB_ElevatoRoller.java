package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_ElevatoRoller extends SubsystemBase {

  private final IO_ElevatorRollerBase io;
  public final ElevatorRollerInputsAutoLogged inputs = new ElevatorRollerInputsAutoLogged();

  public SUB_ElevatoRoller(IO_ElevatorRollerBase io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ElevatorRoller", inputs);
  }

  public void setSpeed(double speed) {
    io.setElevatorRollerSpeed(speed);
  }
}
