package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.subsystems.led.SUB_LED.LEDState;

public class HasCoral extends Command {

  SUB_LED leds;

  public HasCoral(SUB_LED leds) {
    this.leds = leds;
    addRequirements(leds);
  }

  @Override
  public void initialize() {
    leds.setState(LEDState.HAS_CORAL);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
