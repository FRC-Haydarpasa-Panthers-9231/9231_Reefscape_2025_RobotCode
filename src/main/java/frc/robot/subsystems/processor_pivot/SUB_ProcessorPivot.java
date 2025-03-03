package frc.robot.subsystems.processor_pivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SUB_ProcessorPivot extends SubsystemBase {

  private final IO_ProcessorPivotBase io;
  public final ProcessorPivotInputsAutoLogged inputs = new ProcessorPivotInputsAutoLogged();

  private static final LoggedTunableNumber processorPivotDebugSetpoint =
      new LoggedTunableNumber("ProcessorPivot/Processor Pivot Setpoint", 0);

  private static final LoggedTunableNumber processorPivotDebugSpeed =
      new LoggedTunableNumber("ProcessorPivot/Processor Pivot speed", 0.1);

  private double setpointVal;

  private static SysIdRoutine sysIdRoutine;

  public SUB_ProcessorPivot(IO_ProcessorPivotBase io) {
    this.io = io;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(1), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> Logger.recordOutput("SysIDPivotTest", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ProcessorPivot", inputs);
    Logger.recordOutput("Processor_Pivot/setpoint", setpointVal);
    Logger.recordOutput("Processor_Pivot/encoderPosition", io.getProcessorPivotPosition());
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setDebugSpeed(boolean isDirectionPositive) {

    int directionVal = isDirectionPositive ? 1 : -1;
    io.setSpeed(processorPivotDebugSpeed.getAsDouble() * directionVal);
  }

  public void stopMotor() {
    io.stopMotor();
  }

  public double getProcessorPivotPosition() {
    return io.getProcessorPivotPosition();
  }

  public void setPosition(double setPoint) {
    io.setPosition(setPoint);
    setpointVal = setPoint;
  }

  public void setDebugPosition() {

    io.setPosition(processorPivotDebugSetpoint.getAsDouble());
    setpointVal = processorPivotDebugSetpoint.getAsDouble();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
