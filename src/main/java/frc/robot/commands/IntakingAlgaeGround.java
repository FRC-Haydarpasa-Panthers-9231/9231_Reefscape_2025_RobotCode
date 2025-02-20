// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_HEIGHT;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.subsystems.led.SUB_LED.LEDState;
import frc.robot.subsystems.processor_pivot.ProcessorPivotConstants;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.ProcessorRollerConstants;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class IntakingAlgaeGround extends Command {
  /** Creates a new IntakingAlgaeGround. */
  SUB_Elevator elevator;

  SUB_ProcessorRoller processorRoller;
  SUB_ProcessorPivot processorPivot;
  SUB_LED leds;

  public IntakingAlgaeGround(
      SUB_Elevator elevator,
      SUB_ProcessorRoller processorRoller,
      SUB_ProcessorPivot processorPivot,
      SUB_LED leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.processorRoller = processorRoller;
    this.processorPivot = processorPivot;
    this.leds = leds;
    addRequirements(elevator, processorPivot, processorRoller, leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(Meters.of(ELEVATOR_HEIGHT.ALGAE_GROUND_INTAKE.getHeightInMeters()));
    processorRoller.setSpeed(ProcessorRollerConstants.kProcessorRollerGroundIntakeSpeed);

    processorPivot.setPosition(ProcessorPivotConstants.INTAKING_ALGEA_GROUND_POSITION);
    leds.setState(LEDState.INTAKING_ALGEA);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    processorPivot.setPosition(ProcessorPivotConstants.ARM_ZERO_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return processorRoller.hasAlgae();
  }
}
