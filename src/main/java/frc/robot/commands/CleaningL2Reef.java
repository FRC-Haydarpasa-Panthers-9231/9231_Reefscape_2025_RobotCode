// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.subsystems.led.SUB_LED.LEDState;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.ProcessorRollerConstants;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;

public class CleaningL2Reef extends Command {
    SUB_Elevator elevator;
    SUB_ProcessorRoller processorRoller;
    SUB_ProcessorPivot processorPivot;
    SUB_LED leds;

    /** Creates a new CleaningL2Reef. */
    public CleaningL2Reef(SUB_Elevator subElevator,
        SUB_ProcessorRoller processorRoller, SUB_ProcessorPivot processorPivot, SUB_LED leds)
    {
        this.elevator = subElevator;
        this.processorRoller = processorRoller;
        this.processorPivot = processorPivot;
        this.leds = leds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        elevator.setPosition(
            Meters.of(ElevatorConstants.SCORING_HEIGHT.ALGAE_L2_CLEANING.getHeightInMeters()));
        processorRoller.setSpeed(ProcessorRollerConstants.kProcessorRollerIntakeSpeed);
        leds.setState(LEDState.CLEARING_L2);

        // processorPivot.setPosition(Constants.constAlgaeIntake.CLEANING_REEF_L2_PIVOT_POSITION);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        if (processorRoller.hasAlgae()) {
            // processorPivot.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
            // globalElevator.setPosition(Constants.constElevator.PREP_0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
