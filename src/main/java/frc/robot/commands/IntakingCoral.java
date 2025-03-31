package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatorRoller;

public class IntakingCoral extends Command {

  SUB_ElevatorRoller elevatorRoller;

  public IntakingCoral(SUB_ElevatorRoller elevatorRoller) {
    this.elevatorRoller = elevatorRoller;
    addRequirements(elevatorRoller);
  }

  @Override
  public void initialize() {
    elevatorRoller.setSpeed(0.4);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorRoller.stopmotors();
  }

  @Override
  public boolean isFinished() {
    return elevatorRoller.hasCoral();
  }
}
