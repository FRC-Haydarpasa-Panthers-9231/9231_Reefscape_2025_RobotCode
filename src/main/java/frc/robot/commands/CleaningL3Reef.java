// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.ProcessorRollerConstants;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;

public class CleaningL3Reef extends Command {
  // TODO: Drivetraini ekle
  SUB_Elevator elevator;
  SUB_ProcessorRoller processorRoller;
  SUB_ProcessorPivot processorPivot;

  /** Creates a new CleaningL2Reef. */
  public CleaningL3Reef(
      SUB_Elevator subElevator,
      SUB_ProcessorRoller processorRoller,
      SUB_ProcessorPivot processorPivot) {
    this.elevator = subElevator;
    this.processorRoller = processorRoller;
    this.processorPivot = processorPivot;
    addRequirements(elevator, processorRoller, processorPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(ElevatorConstants.ELEVATOR_HEIGHT.ALGAE_L2_CLEANING.getPositionRads());
    processorRoller.setSpeed(ProcessorRollerConstants.kProcessorRollerIntakeSpeed);

    // processorPivot.setPosition(Constants.constAlgaeIntake.CLEANING_REEF_L2_PIVOT_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (processorRoller.hasAlgae()) {
      // processorPivot.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
      // globalElevator.setPosition(Constants.constElevator.PREP_0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
