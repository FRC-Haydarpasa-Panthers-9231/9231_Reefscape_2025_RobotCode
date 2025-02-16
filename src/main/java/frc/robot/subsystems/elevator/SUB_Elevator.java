package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
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

  public void setSpeed(double speed) {
    io.setElevatorSpeed(speed);
  }

  /** Asansör motorlarını durdurur. */
  public void stopElevator() {
    io.stopMotor();
  }
}
