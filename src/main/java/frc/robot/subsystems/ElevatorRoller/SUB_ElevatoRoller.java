package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BeamBreak;
import org.littletonrobotics.junction.Logger;

public class SUB_ElevatoRoller extends SubsystemBase {

    private final IO_ElevatorRollerBase io;
    public final ElevatorRollerInputsAutoLogged inputs = new ElevatorRollerInputsAutoLogged();

    private final BeamBreak beamBreak;
    private boolean hasCoral, indexingCoral;

    public SUB_ElevatoRoller(IO_ElevatorRollerBase io)
    {
        this.io = io;
        beamBreak = new BeamBreak();
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorRoller", inputs);
        Logger.recordOutput("ProcessorRoller/hasCoral", beamBreak.hasCoral());
    }

    public void setSpeed(double speed)
    {
        io.setElevatorRollerSpeed(speed);
    }

    public boolean hasCoral()
    {
        return beamBreak.hasCoral();
    }



}
