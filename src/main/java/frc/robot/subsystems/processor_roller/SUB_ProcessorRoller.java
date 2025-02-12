package frc.robot.subsystems.processor_roller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_ProcessorRoller extends SubsystemBase {

  private final IO_ProcessorRollerBase io;
  public final ProcessorRollerInputsAutoLogged inputs = new ProcessorRollerInputsAutoLogged();

  public enum WantedState {
    OFF,
    SCORE,
    FEED
  }

  public enum SystemState {
    IS_OFF,
    SCORING,
    FEEDING
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.IS_OFF;

  public SUB_ProcessorRoller(IO_ProcessorRollerBase io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    SystemState newState = handleStateTransition();

    if (newState != systemState) {
      Logger.recordOutput("ProcessorRoller/SystemState", newState.toString());
      systemState = newState;
    }

    if (DriverStation.isDisabled()) {
      systemState = SystemState.IS_OFF;
    }

    switch (systemState) {
      case IS_OFF:
        handleOff();
        break;
      case SCORING:
        handleScoring();
        break;
      case FEEDING:
        handleFeeding();
        break;
    }

    Logger.recordOutput("ProcessorRoller/WantedState", wantedState);
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case SCORE -> SystemState.SCORING;
      case FEED -> SystemState.FEEDING;
      default -> SystemState.IS_OFF;
    };
  }

  private void handleOff() {
    io.stopMotor();
  }

  private void handleFeeding() {
    io.setProcessorRollerSpeed(-0.5);
  }

  private void handleScoring() {
    io.setProcessorRollerSpeed(0.5);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void stopMotor() {
    io.stopMotor();
  }
}
