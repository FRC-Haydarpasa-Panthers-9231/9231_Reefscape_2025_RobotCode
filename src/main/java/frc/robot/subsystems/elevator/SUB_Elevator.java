package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.elevator.ElevatorConstants.ReefBranch;
import org.littletonrobotics.junction.Logger;

public class SUB_Elevator extends SubsystemBase {

  private final LoggedTunableNumber elevatorHeightMeters =
      new LoggedTunableNumber("Elevator/Height(meters)", 0);

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

  private WantedState wantedState = WantedState.ZERO_ELEVATOR;
  private SystemState systemState = SystemState.ZEROING;

  public SUB_Elevator(IO_ElevatorBase io) {
    this.io = io;

    io.zeroPosition();

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

  public void runPositonRads(double positionRads) {
    io.runPositionRads(positionRads);
  }

  public void runPositionMeters(double positionMeters) {
    io.runPositionMeters(positionMeters);
  }

  /** Asansör motorlarını durdurur. */
  public void stopElevator() {
    io.stopMotor();
  }

  private Distance reefBranchToDistance(ReefBranch reefBranch) {

    Distance height;

    switch (reefBranch) {
      case L1:
        height = L1_HEIGHT;
        break;

      case L2:
        height = L2_HEIGHT;
        break;

      case L3:
        height = L3_HEIGHT;
        break;

      case L4:
        height = L4_HEIGHT;
        break;

      default:
        height = ElevatorConstants.kMinElevatorHeightMeters;
        break;
    }
    return height;
  }

  public boolean isAtPosition(ReefBranch reefBranch) {

    return getPosition().minus(reefBranchToDistance(reefBranch)).abs(Meters) < TOLERANCE_METERS;
  }

  public Distance getPosition() {
    return Meters.of(inputs.positionMeters);
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
    io.runPositionMeters(0);
  }

  private void handleIdling() {
    io.runPositionMeters(0.3);
  }

  private void handleCoralStage1() {
    io.runPositionMeters(0.6);
  }

  private void handleCoralStage2() {
    io.runPositionMeters(0.9);
  }

  private void handleCoralStage3() {
    io.runPositionMeters(1.2);
  }

  private void handleCoralStage4() {
    io.runPositionMeters(1.4);
  }

  private void handleAlgeaStage1() {
    io.runPositionMeters(30);
  }

  private void handleAlgeaStage2() {
    io.runPositionMeters(60);
  }

  private void handleAlgeaGround() {
    io.runPositionMeters(0);
  }

  private void handleAlgeaProcessor() {
    io.runPositionMeters(0);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setStateCommand(WantedState state) {
    return runOnce(() -> this.wantedState = state);
  }
}
