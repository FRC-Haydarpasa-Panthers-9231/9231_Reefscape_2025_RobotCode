package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;

public class DebugIntaking extends Command {

  SUB_ElevatoRoller elevatorRoller;

  public DebugIntaking(SUB_ElevatoRoller elevatorRoller) {
    this.elevatorRoller = elevatorRoller;
    addRequirements(elevatorRoller);
  }

  @Override
  public void initialize() {
    elevatorRoller.setSpeed(0.5);
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
