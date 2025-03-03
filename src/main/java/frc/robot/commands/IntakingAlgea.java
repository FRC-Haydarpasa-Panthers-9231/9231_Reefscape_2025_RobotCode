package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.processor_roller.ProcessorRollerConstants;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;

public class IntakingAlgea extends Command {

  SUB_ProcessorRoller processorRoller;

  public IntakingAlgea(SUB_ProcessorRoller processorRoller) {

    this.processorRoller = processorRoller;
    addRequirements(processorRoller);
  }

  @Override
  public void initialize() {
    processorRoller.setSpeed(ProcessorRollerConstants.kProcessorRollerIntakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    processorRoller.stopMotor();
    // processorPivot.setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return processorRoller.hasAlgae();
  }
}
