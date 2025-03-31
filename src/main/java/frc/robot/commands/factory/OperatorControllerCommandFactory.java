package frc.robot.commands.factory;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorConsole;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;

public class OperatorControllerCommandFactory {

  private OperatorControllerCommandFactory() {}

  public static void registerCommands(
      OperatorConsole operatorController,
      SUB_Elevator elevator,
      SUB_ElevatoRoller elevatoRoller,
      SUB_ProcessorPivot processorPivot,
      SUB_ProcessorRoller processorRoller) {

        operatorController.getElevatorA1PositionButton().onTrue(
          Commands.run(null, null)
        );


        operatorController.getElevatorA2PositionButton().onTrue(
          Commands.run(null, null)
        );


        operatorController.getElevatorZeroPositionButton().onTrue(
          Commands.run(null, null)
        );


        operatorController.getElevatorL2PositionButton().onTrue(
          Commands.run(null, null)
        );


        operatorController.getElevatorL3PositionButton().onTrue(
          Commands.run(null, null)
        );


        operatorController.getElevatorL4PositionButton().onTrue(
          Commands.run(null, null)
        );


        operatorController.getIntakingCoralButton().onTrue(
          Commands.run(null, null)
        );


        operatorController.getScoringCoralButton().whileTrue(
          Commands.run( , null)
        );






      }
}
