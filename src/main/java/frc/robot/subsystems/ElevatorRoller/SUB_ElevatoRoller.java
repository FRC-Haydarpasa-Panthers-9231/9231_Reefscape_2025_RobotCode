package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_ElevatoRoller extends SubsystemBase {

  private final IO_ElevatorRollerBase io;
  public final ElevatorRollerInputsAutoLogged inputs = new ElevatorRollerInputsAutoLogged();

  public enum WantedState {
    ZERO_ELEVATOR
  }

  public enum SystemState {
    ZEROING
  }

  private WantedState wantedState = WantedState.ZERO_ELEVATOR;
  private SystemState systemState = SystemState.ZEROING;

  public SUB_ElevatoRoller(IO_ElevatorRollerBase io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    SystemState newState = handleStateTransition();

    if (newState != systemState) {
      Logger.recordOutput("ElevatorRoller/SystemState", newState.toString());
      systemState = newState;
    }

    if (DriverStation.isDisabled()) {
      // TODO: kapalı durumdaki state'i yaz
      // systemState = SystemState.ZEROING;
    }

    switch (systemState) {
      case ZEROING:
        handleZeroing();
        break;
    }

    Logger.recordOutput("ElevatorRoller/WantedState", wantedState);
  }

  @Override
  public void simulationPeriodic() {}

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      default -> SystemState.ZEROING;
    };
  }

  private void handleZeroing() {
    // io.runPosition(0);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
