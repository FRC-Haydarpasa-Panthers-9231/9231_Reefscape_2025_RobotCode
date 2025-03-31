package frc.lib.team9231;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
  private final DigitalInput limitSwitch;

  public LimitSwitch(int port) {
    limitSwitch = new DigitalInput(port);
  }

  public boolean getValue() {
    return limitSwitch.get();
  }
}
