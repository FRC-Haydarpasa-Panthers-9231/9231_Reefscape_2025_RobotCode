package frc.robot.subsystems.sensors;

import frc.lib.team9231.BeamBreak;

public class CoralSensorIOBeam implements CoralSensorIO {

  private BeamBreak sensor;

  public CoralSensorIOBeam(int id) {
    sensor = new BeamBreak(id);
  }

  public boolean getValue() {
    return sensor.getValue();
  }

  @Override
  public void updateInputs(CoralSensorIOInputs inputs) {
    inputs.data = new CoralSensorIOData(sensor.getValue());
  }
}
