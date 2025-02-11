package frc.robot.subsystems.processor_pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ProcessorPivotBase {

  @AutoLog
  public static class ProcessorPivotInputs {
    public double processorPivotAppliedVolts = 0.0;
    public double processorPivotCurrentAmps = 0.0;
    public double elevatorPositionRad = 0.0;
  }

  void updateInputs(ProcessorPivotInputs inputs);

  public double getProcessorPivotPosition();

  public void stopMotor();

  public void setMotorSpeed(double speed);

  public void setPosition(double setPoint);

  public void setPID(double kP, double kI, double kD);
}
