package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorLimitSwitchIO {
  @AutoLog
  class LimitSwitchSensorIOInputs {
    public LimitSwitchSensorIOData data = new LimitSwitchSensorIOData(false);
  }

  public boolean getValue();

  record LimitSwitchSensorIOData(boolean isTriggered) {}

  default void updateInputs(LimitSwitchSensorIOInputs inputs) {}
}
