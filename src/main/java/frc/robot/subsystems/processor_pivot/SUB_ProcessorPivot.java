package frc.robot.subsystems.processor_pivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SUB_ProcessorPivot extends SubsystemBase {

  private final IO_ProcessorPivotBase io;
  public final ProcessorPivotInputsAutoLogged inputs = new ProcessorPivotInputsAutoLogged();

  // private static final LoggedTunableNumber kG = new LoggedTunableNumber("Processor_Pivot/kG",
  // 0);
  // private static final LoggedTunableNumber kS = new LoggedTunableNumber("Processor_Pivot/kS",
  // 0);
  // private static final LoggedTunableNumber kV = new LoggedTunableNumber("Processor_Pivot/kV",
  // 0);
  // private static final LoggedTunableNumber kA = new LoggedTunableNumber("Processor_Pivot/kA",
  // 0);
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Processor_Pivot/kP", 0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Processor_Pivot/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Processor_Pivot/kD", 0);

  // private final ArmFeedforward ff =
  // new ArmFeedforward(kS.getAsDouble(), kG.getAsDouble(),
  // kV.getAsDouble(),
  // kA.getAsDouble());

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
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }

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

  public void setMotorSpeed(double speed) {
    io.setMotorSpeed(speed);
  }

  public void stopMotor() {
    io.stopMotor();
  }

  public double getProcessorPivotPosition() {
    return io.getProcessorPivotPosition();
  }

  public void setPosition(double setPoint) {
    io.setPosition(setPoint);
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      default -> SystemState.ZEROING;
    };
  }

  private void handleZeroing() {}

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setStateCommand(WantedState state) {
    return runOnce(() -> this.wantedState = state);
  }
}
