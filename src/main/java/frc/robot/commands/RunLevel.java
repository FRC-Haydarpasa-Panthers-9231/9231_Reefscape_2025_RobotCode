package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.subsystems.led.SUB_LED.LEDState;

public class RunLevel extends Command {

  SUB_Elevator elevator;
  double distance;
  SUB_LED leds;

  public RunLevel(SUB_Elevator elevator, SUB_LED leds, double height) {
    this.elevator = elevator;
    this.distance = height;
    this.leds = leds;
    addRequirements(elevator, leds);
  }

  @Override
  public void initialize() {
    elevator.setPosition(distance);
    leds.setState(LEDState.MOVING_ELEVATOR);
    System.out.println("çalisdi");
  }

  @Override
  public boolean isFinished() {
    return elevator.isAtSetPoint();
  }
}
