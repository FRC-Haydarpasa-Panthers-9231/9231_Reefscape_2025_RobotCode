package frc.lib.team9231;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {
  private final DigitalInput photoelectricSensor;

  public BeamBreak(int id) {
    photoelectricSensor = new DigitalInput(id);
  }

  public boolean getValue() {
    return photoelectricSensor.get();
  }
}
