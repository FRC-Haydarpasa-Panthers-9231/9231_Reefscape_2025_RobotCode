package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SUB_Elevator extends SubsystemBase {

  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0);
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 50);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 1);
  private static final LoggedTunableNumber elevatorGoalPosition =
      new LoggedTunableNumber("Elevator/elevatorGoalPosition", 0);

  private static SysIdRoutine sysIdRoutine;

  private final IO_ElevatorBase io;
  public final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  public enum WantedState {
    OFF,
    ZERO_ELEVATOR,
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

  public enum SystemState {
    IS_OFF,
    ZEROING,
    IDLING,
    CORAL_STAGE_1,
    CORAL_STAGE_2,
    CORAL_STAGE_3,
    CORAL_STAGE_4,
    ALGEA_STAGE_1,
    ALGEA_STAGE_2,
    ALGEA_GROUND,
    ALGEA_PROCESSOR,
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.ZEROING;

  public SUB_Elevator(IO_ElevatorBase io) {
    this.io = io;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setElevatorVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Tuning mode açık ise değerleri elastic dashboard üzerinden değiştirip deneyebiliriz.
    if (kG.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())) {
      io.setSlot0(kG.get(), kS.get(), kV.get(), kA.get(), kP.get(), kI.get(), kD.get());
    }

    SystemState newState = handleStateTransition();

    // Eğer yeni state eski state ile aynı değilse
    if (newState != systemState) {
      // state'i günceller
      Logger.recordOutput("Elevator/SystemState", newState.toString());
      systemState = newState;
    }

    // eğer robot kapalıysa
    if (DriverStation.isDisabled()) {
      // IS_OFF state'i motorları durdurur
      systemState = SystemState.IS_OFF;
    }

    // IMPORTANT:
    switch (systemState) {
      case IS_OFF:
        handleIsOff();
        break;

      case ZEROING:
        handleZeroing();
        break;

      case IDLING:
        handleIdling();
        break;

      case CORAL_STAGE_1:
        handleCoralStage1();
        break;

      case CORAL_STAGE_2:
        handleCoralStage2();
        break;

      case CORAL_STAGE_3:
        handleCoralStage3();
        break;

      case CORAL_STAGE_4:
        handleCoralStage4();
        break;

      case ALGEA_STAGE_1:
        handleAlgeaStage1();
        break;

      case ALGEA_STAGE_2:
        handleAlgeaStage2();
        break;

      case ALGEA_GROUND:
        handleAlgeaGround();
        break;

      case ALGEA_PROCESSOR:
        handleAlgeaProcessor();
        break;
    }

    Logger.recordOutput("Elevator/WantedState", wantedState);
  }

  /**
   * Asansör motorlarının voltajını ayarlar
   *
   * @param volts Çalıştırmak istediğiniz volt değeri.
   */
  public void setElevatorVoltage(double volts) {
    io.setElevatorVoltage(volts);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Asansörü belli bir pozisyona götürür.
   *
   * @param positionRad başlangıç konumundan motorun radyan cinsinden sabitlenmek istendiği konum.
   */
  public void runPosition(double positionRad) {
    io.runPosition(positionRad);
  }

  /** Asansör motorlarını durdurur. */
  public void stopElevator() {
    io.stopMotor();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case ZERO_ELEVATOR -> SystemState.ZEROING;
      case IDLE -> SystemState.IDLING;
      case CORAL_STAGE_1 -> SystemState.CORAL_STAGE_1;
      case CORAL_STAGE_2 -> SystemState.CORAL_STAGE_2;
      case CORAL_STAGE_3 -> SystemState.CORAL_STAGE_3;
      case CORAL_STAGE_4 -> SystemState.CORAL_STAGE_4;
      case ALGEA_STAGE_1 -> SystemState.ALGEA_STAGE_1;
      case ALGEA_STAGE_2 -> SystemState.ALGEA_STAGE_2;
      case ALGEA_GROUND -> SystemState.ALGEA_GROUND;
      case ALGEA_PROCESSOR -> SystemState.ALGEA_PROCESSOR;
      default -> SystemState.IS_OFF;
    };
  }

  private void handleIsOff() {
    stopElevator();
  }

  private void handleZeroing() {
    io.runPosition(0);
  }

  private void handleIdling() {
    io.runPosition(20);
  }

  private void handleCoralStage1() {
    io.runPosition(40);
  }

  private void handleCoralStage2() {
    io.runPosition(60);
  }

  private void handleCoralStage3() {
    io.runPosition(80);
  }

  private void handleCoralStage4() {
    io.runPosition(100);
  }

  private void handleAlgeaStage1() {
    io.runPosition(30);
  }

  private void handleAlgeaStage2() {
    io.runPosition(60);
  }

  private void handleAlgeaGround() {
    io.runPosition(0);
  }

  private void handleAlgeaProcessor() {
    io.runPosition(0);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
