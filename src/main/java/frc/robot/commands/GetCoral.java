package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorRoller.ElevatorRollerConstants;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;
import frc.robot.subsystems.elevator.SUB_Elevator;

public class GetCoral extends Command {

  SUB_ElevatoRoller elevatorRoller;
  SUB_Elevator elevator;

  boolean hasTimeout = false;
  Time timeout;

  public GetCoral(SUB_ElevatoRoller elevatorRoller, SUB_Elevator elevator) {
    this.elevatorRoller = elevatorRoller;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    timeout =
        Units.Seconds.of(
            Timer.getFPGATimestamp() + ElevatorRollerConstants.kFeedTimeout.magnitude());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    // Timeout kontrolü
    if (Units.Seconds.of(Timer.getFPGATimestamp()).gte(timeout)) {
      return true;
    }
    return elevatorRoller.hasCoral();
  }
}
