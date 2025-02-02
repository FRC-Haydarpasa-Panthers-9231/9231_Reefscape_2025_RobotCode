package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ElevatorBase {

  @AutoLog
  public static class ElevatorInputs {
    public double[] elevatorAppliedVolts = new double[] {};
    public double[] elevatorCurrentAmps = new double[] {};
    public double[] elevatorTempCelcius = new double[] {};
    public double elevatorPositionRad = 0.0;
    public double elevatorVelocityRadPerSec = 0.0;
  }

  void updateInputs(ElevatorInputs inputs);

  public void setElevatorVoltage(double volts);

  public void setSlot0(double kG, double kS, double kV, double kA, double kP, double kI, double kD);

  public void stopMotor();

  public void runPosition(double positionRad);

  public boolean isPositionWithinLimits(double positionRad);
}
