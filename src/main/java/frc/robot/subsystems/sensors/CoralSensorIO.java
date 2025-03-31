package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
  @AutoLog
  class CoralSensorIOInputs {
    public CoralSensorIOData data = new CoralSensorIOData(false);
  }

  record CoralSensorIOData(boolean isTriggered) {}

  public boolean getValue();

  default void updateInputs(CoralSensorIOInputs inputs) {}
}
