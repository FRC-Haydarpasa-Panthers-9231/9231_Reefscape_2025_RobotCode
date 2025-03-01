package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.SUB_Elevator;

public class LimitSwitchZeroing extends Command {

  SUB_Elevator elevator;

  public LimitSwitchZeroing(SUB_Elevator elevator) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setPosition(-0.2);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setEncoderPosition(0);
    elevator.setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return elevator.limitSwitchvalue();
  }
}
