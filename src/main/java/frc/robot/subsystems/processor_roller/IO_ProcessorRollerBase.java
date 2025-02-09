package frc.robot.subsystems.processor_roller;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ProcessorRollerBase {

  @AutoLog
  public static class ProcessorRollerInputs {
    public double processorRollerAppliedVolts = 0.0;
    public double processorRollerCurrentAmps = 0.0;
  }

  void updateInputs(ProcessorRollerInputs inputs);

  public void setProcessorRollerVoltage(double speed);

  public void stopMotor();
}
