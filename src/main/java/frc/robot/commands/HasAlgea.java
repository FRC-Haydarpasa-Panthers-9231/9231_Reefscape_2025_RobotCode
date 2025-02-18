package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.subsystems.led.SUB_LED.LEDState;
import frc.robot.subsystems.processor_roller.ProcessorRollerConstants;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;

public class HasAlgea extends Command {

    SUB_ProcessorRoller processorRoller;
    SUB_LED leds;

    public HasAlgea(SUB_ProcessorRoller processorRoller, SUB_LED leds)
    {

        this.processorRoller = processorRoller;
        this.leds = leds;
    }

    @Override
    public void initialize()
    {
        leds.setState(LEDState.HAS_ALGEA);
    }


    @Override
    public void end(boolean interrupted)
    {
        processorRoller.setAlgaeIntakeVoltage(ProcessorRollerConstants.kHoldalgeaIntakeVoltage);
    }

    @Override
    public boolean isFinished()
    {

            return true;

    }


}
