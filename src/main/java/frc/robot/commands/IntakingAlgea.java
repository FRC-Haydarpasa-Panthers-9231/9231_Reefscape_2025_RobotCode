package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.ProcessorRollerConstants;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;

public class IntakingAlgea extends Command {

  SUB_ProcessorRoller processorRoller;
  SUB_ProcessorPivot processorPivot;

  public IntakingAlgea(SUB_ProcessorRoller processorRoller, SUB_ProcessorPivot processorPivot) {

    this.processorRoller = processorRoller;
    this.processorPivot = processorPivot;
    addRequirements(processorPivot, processorRoller);
  }

  @Override
  public void initialize() {
    processorRoller.setSpeed(ProcessorRollerConstants.kProcessorRollerIntakeSpeed);
    // processorPivot.setPosition(0);
  }

  @Override
  public void end(boolean interrupted) {
    // processorPivot.setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return processorRoller.hasAlgae();
  }
}
