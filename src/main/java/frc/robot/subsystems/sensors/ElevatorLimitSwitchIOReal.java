package frc.robot.subsystems.sensors;

import frc.lib.team9231.LimitSwitch;

public class ElevatorLimitSwitchIOReal implements ElevatorLimitSwitchIO {

  private LimitSwitch sensor;

  public ElevatorLimitSwitchIOReal(int id) {
    sensor = new LimitSwitch(id);
  }

  public boolean getValue() {
    return sensor.getValue();
  }

  @Override
  public void updateInputs(LimitSwitchSensorIOInputs inputs) {
    inputs.data = new LimitSwitchSensorIOData(sensor.getValue());
  }
}
