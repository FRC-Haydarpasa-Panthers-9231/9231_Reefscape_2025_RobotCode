package frc.robot.subsystems.processor_pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ProcessorPivotBase {

  @AutoLog
  public static class ProcessorPivotInputs {
    public double[] elevatorAppliedVolts = new double[] {};
    public double[] elevatorCurrentAmps = new double[] {};
    public double elevatorPositionRad = 0.0;
    public double elevatorVelocityRadPerSec = 0.0;
  }

  void updateInputs(ProcessorPivotInputs inputs);
}
