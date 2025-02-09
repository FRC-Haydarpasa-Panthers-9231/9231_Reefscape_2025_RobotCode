package frc.robot.subsystems.processor_pivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_ProcessorPivot extends SubsystemBase {

  private final IO_ProcessorPivotBase io;
  public final ProcessorPivotInputsAutoLogged inputs = new ProcessorPivotInputsAutoLogged();

  public enum WantedState {
    ZERO_Processor
  }

  public enum SystemState {
    ZEROING
  }

  private WantedState wantedState = WantedState.ZERO_Processor;
  private SystemState systemState = SystemState.ZEROING;

  public SUB_ProcessorPivot(IO_ProcessorPivotBase io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    SystemState newState = handleStateTransition();

    if (newState != systemState) {
      Logger.recordOutput("ProcessorPivot/SystemState", newState.toString());
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

    Logger.recordOutput("ProcessorPivot/WantedState", wantedState);
  }

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
