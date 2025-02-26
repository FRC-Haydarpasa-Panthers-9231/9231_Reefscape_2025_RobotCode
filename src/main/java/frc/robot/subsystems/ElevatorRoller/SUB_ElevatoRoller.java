package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SUB_ElevatoRoller extends SubsystemBase {

    private final IO_ElevatorRollerBase io;
    public final ElevatorRollerInputsAutoLogged inputs = new ElevatorRollerInputsAutoLogged();
    private static final LoggedTunableNumber elevatorRollerDebugSpeed =
        new LoggedTunableNumber("ElevatorRoller/Elevator Roller speed", 0.3);

    public SUB_ElevatoRoller(IO_ElevatorRollerBase io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorRoller", inputs);
        Logger.recordOutput("ElevatorRoller/hasCoral", hasCoral());
    }

    public void setSpeed(double speed)
    {
        io.setElevatorRollerSpeed(speed);
    }

    public void setDebugSpeed(boolean isForward)
    {
        double speed =
            isForward
                ? 1 * elevatorRollerDebugSpeed.getAsDouble()
                : -1 * elevatorRollerDebugSpeed.getAsDouble();
        io.setElevatorRollerSpeed(speed);
    }

    public boolean hasCoral()
    {
        return io.hasCoral();
    }

    public void stopmotors()
    {
        io.stopMotors();
    }
}
