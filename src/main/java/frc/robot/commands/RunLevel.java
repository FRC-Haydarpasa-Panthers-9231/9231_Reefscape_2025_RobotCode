package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.SUB_Elevator;

public class RunLevel extends Command {

    SUB_Elevator elevator;
    double distance;

    public RunLevel(SUB_Elevator elevator, double height)
    {
        this.elevator = elevator;
        this.distance = height;
        addRequirements(elevator);
    }

    @Override
    public void initialize()
    {
        elevator.setPosition(distance);
    }

    @Override
    public boolean isFinished()
    {
        return elevator.isAtSetPoint();
    }
}
