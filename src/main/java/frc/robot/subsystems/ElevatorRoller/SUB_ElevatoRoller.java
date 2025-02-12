package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_ElevatoRoller extends SubsystemBase {

  private final IO_ElevatorRollerBase io;
  public final ElevatorRollerInputsAutoLogged inputs = new ElevatorRollerInputsAutoLogged();

  public enum WantedState {
    OFF,
    OUTTAKE,
  }

  public enum SystemState {
    IS_OFF,
    OUTTAKING
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.IS_OFF;

  public SUB_ElevatoRoller(IO_ElevatorRollerBase io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    SystemState newState = handleStateTransition();

    // Eğer yeni state eski state ile aynı değilse
    if (newState != systemState) {
      // state'i günceller
      Logger.recordOutput("ElevatorRoller/SystemState", newState.toString());
      systemState = newState;
    }

    // eğer robot kapalıysa
    if (DriverStation.isDisabled()) {
      // IS_OFF state'i motorları durdurur
      systemState = SystemState.IS_OFF;
    }

    // State durumunda ne yapılacağını ayarlar.
    switch (systemState) {
      case IS_OFF:
        handleIsOff();
        break;
      case OUTTAKING:
        handleOuttaking();
        break;
    }

    Logger.recordOutput("ElevatorRoller/WantedState", wantedState);
  }

  /**
   * State geçişlerini ayarlar. Ne kadar state varsa burada olmak zorunda.
   *
   * @return System State döndürür
   */
  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case OFF -> SystemState.OUTTAKING;
      default -> SystemState.IS_OFF;
    };
  }

  // ElevatorRoller motorlarını durdurur
  private void handleIsOff() {
    io.stopMotors();
  }

  // ElevatorRoller'ın hızını ayarlar.
  private void handleOuttaking() {
    io.setElevatorRollerSpeed(0.5);
  }

  /**
   * @param wantedState Elevator Roller'ın olması istenen durumu
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
