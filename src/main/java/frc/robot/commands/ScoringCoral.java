package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorRoller.ElevatorRollerConstants;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_HEIGHT;
import frc.robot.subsystems.elevator.SUB_Elevator;

public class ScoringCoral extends SequentialCommandGroup {

  SUB_ElevatoRoller elevatorRoller;
  SUB_Elevator elevator;
  ELEVATOR_HEIGHT scoringHeight;
  double coralOuttakeSpeed;

  public ScoringCoral(
      SUB_Elevator elevator, SUB_ElevatoRoller elevatorRoller, ELEVATOR_HEIGHT scoringHeight) {
    this.elevatorRoller = elevatorRoller;
    this.elevator = elevator;
    this.scoringHeight = scoringHeight;

    addCommands(
        Commands.runOnce(() -> elevator.setPosition(scoringHeight.getPositionRads()), elevator),
        // Asansör dogru konuma geldiginde atış yap
        Commands.waitUntil(() -> elevator.isAtSetPoint()),
        Commands.runOnce(() -> elevatorRoller.setSpeed(getCoralOuttakeSpeed()), elevatorRoller),
        // Commands.waitUntil(() -> !elevatorRoller.hasCoral()),
        Commands.waitSeconds(ElevatorRollerConstants.kCoralScoreTime.in(Units.Seconds)),
        Commands.runOnce(() -> elevatorRoller.setSpeed(0), elevatorRoller),
        Commands.runOnce(
            () ->
                elevator.setPosition(
                    ElevatorConstants.ELEVATOR_HEIGHT.ZERO_HEIGHT.getPositionRads())));

    addRequirements(elevator, elevatorRoller);
  }

  public double getCoralOuttakeSpeed() {
    if (scoringHeight.equals(ELEVATOR_HEIGHT.CORAL_L1_HEIGHT)) {
      coralOuttakeSpeed = ElevatorRollerConstants.kL1ScoringSpeed;
    } else if (scoringHeight.equals(ELEVATOR_HEIGHT.CORAL_L2_HEIGHT)) {
      coralOuttakeSpeed = ElevatorRollerConstants.kL2ScoringSpeed;
    } else if (scoringHeight.equals(ELEVATOR_HEIGHT.CORAL_L3_HEIGHT)) {
      coralOuttakeSpeed = ElevatorRollerConstants.kL3ScoringSpeed;
    } else {
      coralOuttakeSpeed = ElevatorRollerConstants.kL4ScoringSpeed;
    }

    return coralOuttakeSpeed;
  }
}
