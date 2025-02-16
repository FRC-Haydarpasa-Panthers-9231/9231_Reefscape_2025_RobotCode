// Gerekli WPILib sınıfları ve robot alt sistemleri dahil edilir
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private SUB_Elevator elevator;
  private SUB_ElevatoRoller elevatorRoller;
  private final SUB_ProcessorPivot processorPivot;
  private final SUB_ProcessorRoller processorRoller;
  private SUB_LED led;
  private Drive swerve;
  private RobotContainer container;

  // Superstructure için istenilen durumlar tanımlanır
  public enum WantedSuperState {
    ZERO_STATE, // Tüm bileşenlerin sıfırlandığı durum
    IDLE, // Bekleme durumu
    CORAL_STAGE_1,
    CORAL_STAGE_2,
    CORAL_STAGE_3,
    CORAL_STAGE_4,
    ALGEA_STAGE_1,
    ALGEA_STAGE_2,
    ALGEA_GROUND,
    ALGEA_PROCESSOR,
  }

  // Superstructure’ın current durumları tanımlanır
  public enum CurrentSuperState {
    ZERO_STATE,
    IDLE,
    CORAL_STAGE_1,
    CORAL_STAGE_2,
    CORAL_STAGE_3,
    CORAL_STAGE_4,
    ALGEA_STAGE_1,
    ALGEA_STAGE_2,
    ALGEA_GROUND,
    ALGEA_PROCESSOR,
  }

  private WantedSuperState wantedSuperState = WantedSuperState.ZERO_STATE;
  private CurrentSuperState currentSuperState = CurrentSuperState.ZERO_STATE;
  private CurrentSuperState previousSuperState;

  public Superstructure(
      Drive swerve,
      SUB_Elevator elevator,
      SUB_ElevatoRoller elevatoRoller,
      SUB_ProcessorPivot processorPivot,
      SUB_ProcessorRoller processorRoller,
      SUB_LED led,
      RobotContainer container) {
    this.swerve = swerve;
    this.elevator = elevator;
    this.elevatorRoller = elevatoRoller;
    this.processorPivot = processorPivot;
    this.processorRoller = processorRoller;
    this.led = led;
    this.container = container;
  }

  @Override
  public void periodic() {
    // current durumu kontrol et ve gerekli geçişleri yap
    currentSuperState = handleStateTransitions();
    // Duruma özel stateleri uygula
    applyStates();

    // wanted ve current state'i kaydet
    Logger.recordOutput("DesiredSuperstate", wantedSuperState);
    if (currentSuperState != previousSuperState) {
      Logger.recordOutput("CurrentSuperstate", currentSuperState);
    }
  }

  // wanted duruma göre geçiş yapılması gereken current durumu belirler
  private CurrentSuperState handleStateTransitions() {
    previousSuperState = currentSuperState; // Önceki durumu saklar
    switch (wantedSuperState) {
      case ZERO_STATE:
        currentSuperState = CurrentSuperState.ZERO_STATE; // Tüm bileşenler sıfırlanır
        break;

      case IDLE:
        currentSuperState = CurrentSuperState.IDLE; // Bekleme durumuna geçilir
        break;

      case CORAL_STAGE_1:
        currentSuperState = CurrentSuperState.CORAL_STAGE_1; // Mercan aşaması 1’e geçilir
        break;

      case CORAL_STAGE_2:
        currentSuperState = CurrentSuperState.CORAL_STAGE_2; // Mercan aşaması 2’ye geçilir
        break;

      case CORAL_STAGE_3:
        currentSuperState = CurrentSuperState.CORAL_STAGE_3; // Mercan aşaması 3’e geçilir
        break;

      case CORAL_STAGE_4:
        currentSuperState = CurrentSuperState.CORAL_STAGE_4; // Mercan aşaması 4’e geçilir
        break;

      case ALGEA_STAGE_1:
        currentSuperState = CurrentSuperState.ALGEA_STAGE_1; // Alg aşaması 1’e geçilir
        break;

      case ALGEA_STAGE_2:
        currentSuperState = CurrentSuperState.ALGEA_STAGE_2; // Alg aşaması 2’ye geçilir
        break;

      case ALGEA_GROUND:
        currentSuperState = CurrentSuperState.ALGEA_GROUND; // Alg yerden alma aşamasına
        // geçilir
        break;

      case ALGEA_PROCESSOR:
        currentSuperState = CurrentSuperState.ALGEA_PROCESSOR; // Alg işleyici aşamasına
        // geçilir
        break;

      default:
        currentSuperState = CurrentSuperState.ZERO_STATE; // Varsayılan durum sıfırlanır
    }
    return currentSuperState;
  }

  // Belirlenen duruma uygun işlemleri uygular
  private void applyStates() {
    switch (currentSuperState) {
      case ZERO_STATE:
        handleZeroStates(); // Sıfırlama işlemleri
        break;

      case IDLE:
        handleIdling(); // Bekleme işlemleri
        break;

      case CORAL_STAGE_1:
        handleCoralStage1(); // Mercan aşaması 1 işlemleri
        break;

      case CORAL_STAGE_2:
        handleCoralStage2(); // Mercan aşaması 2 işlemleri
        break;

      case CORAL_STAGE_3:
        handleCoralStage3(); // Mercan aşaması 3 işlemleri
        break;

      case CORAL_STAGE_4:
        handleCoralStage4(); // Mercan aşaması 4 işlemleri
        break;

      case ALGEA_STAGE_1:
        handleAlgeaStage1(); // Alg aşaması 1 işlemleri
        break;

      case ALGEA_STAGE_2:
        handleAlgeaStage2(); // Alg aşaması 2 işlemleri
        break;

      case ALGEA_GROUND:
        handleAlgeaGround(); // Alg yerden alma işlemleri
        break;

      case ALGEA_PROCESSOR:
        handleAlgeaProcessor(); // Alg işleyici işlemleri
        break;
    }
  }

  // Sıfırlama işlemleri
  private void handleZeroStates() {
    elevator.setWantedState(SUB_Elevator.WantedState.ZERO_ELEVATOR);
    processorPivot.setWantedState(SUB_ProcessorPivot.WantedState.ZERO_Processor);
    elevatorRoller.setWantedState(SUB_ElevatoRoller.WantedState.OFF);
    processorRoller.setWantedState(SUB_ProcessorRoller.WantedState.OFF);
    led.setState(SUB_LED.LEDState.RAINBOW);
  }

  // Bekleme işlemleri
  private void handleIdling() {
    elevator.setWantedState(SUB_Elevator.WantedState.ZERO_ELEVATOR);
    processorRoller.setWantedState(SUB_ProcessorRoller.WantedState.OFF);
  }

  // Mercan aşamaları işlemleri
  private void handleCoralStage1() {
    elevator.setWantedState(SUB_Elevator.WantedState.CORAL_STAGE_1);
  }

  private void handleCoralStage2() {
    elevator.setWantedState(SUB_Elevator.WantedState.CORAL_STAGE_2);
  }

  private void handleCoralStage3() {
    elevator.setWantedState(SUB_Elevator.WantedState.CORAL_STAGE_3);
  }

  private void handleCoralStage4() {
    elevator.setWantedState(SUB_Elevator.WantedState.CORAL_STAGE_4);
  }

  // Alg aşamaları işlemleri
  private void handleAlgeaStage1() {
    elevator.setWantedState(SUB_Elevator.WantedState.ALGEA_STAGE_1);
  }

  private void handleAlgeaStage2() {
    elevator.setWantedState(SUB_Elevator.WantedState.ALGEA_STAGE_2);
  }

  private void handleAlgeaGround() {
    elevator.setWantedState(SUB_Elevator.WantedState.ALGEA_GROUND);
  }

  private void handleAlgeaProcessor() {
    elevator.setWantedState(SUB_Elevator.WantedState.ALGEA_PROCESSOR);
  }

  /**
   * @param wantedSuperState İstenilen durumu ayarlar
   */
  public void setWantedSuperState(WantedSuperState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
  }

  /**
   * İstenilen durumu komut içinde ayarlar.
   *
   * @param wantedSuperState istenilen durum
   * @return Komut döndürür.
   */
  public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
  }
}
