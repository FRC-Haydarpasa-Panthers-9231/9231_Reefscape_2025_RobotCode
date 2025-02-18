package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.ProcessorRollerConstants;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;

public class ScoringAlgea extends Command {
    SUB_ProcessorPivot processorPivot;
    SUB_ProcessorRoller processorRoller;
    SUB_Elevator elevator;
    SUB_LED leds;

    public ScoringAlgea(SUB_ProcessorPivot processorPivot, SUB_ProcessorRoller processorRoller,
        SUB_Elevator elevator, SUB_LED leds)
    {

        this.processorPivot = processorPivot;
        this.processorRoller = processorRoller;
        this.elevator = elevator;
        this.leds = leds;
    }

    @Override
    public void initialize()
    {
        leds.setState(SUB_LED.LEDState.SCORING_ALGEA);
        // processorPivot.setPosition(0);
    }

    @Override
    public void execute()
    {
        if (elevator.isAtSetPoint()) {
            processorRoller.setSpeed(ProcessorRollerConstants.kProcessorRollerScoringSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        processorRoller.setSpeed(0);
        processorRoller.setHasAlgaeOverride(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
