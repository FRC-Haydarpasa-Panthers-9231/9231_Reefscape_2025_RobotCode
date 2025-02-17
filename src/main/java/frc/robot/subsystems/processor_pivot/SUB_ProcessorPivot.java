package frc.robot.subsystems.processor_pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SUB_ProcessorPivot extends SubsystemBase {

    private final IO_ProcessorPivotBase io;
    public final ProcessorPivotInputsAutoLogged inputs = new ProcessorPivotInputsAutoLogged();

    // FIXME: DOĞRU PID DEĞERLERİNİ BUL
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Processor_Pivot/kP", 0);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Processor_Pivot/kI", 0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Processor_Pivot/kD", 0);
    private double setpointVal;

    public SUB_ProcessorPivot(IO_ProcessorPivotBase io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs("ProcessorPivot", inputs);
        Logger.recordOutput("Processor_Pivot/setpoint", setpointVal);
        Logger.recordOutput("Processor_Pivot/encoderPosition", io.getProcessorPivotPosition());

        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPID(kP.get(), kI.get(), kD.get());
        }
    }

    public void setSpeed(double speed)
    {
        io.setSpeed(speed);
    }

    public void stopMotor()
    {
        io.stopMotor();
    }

    public double getProcessorPivotPosition()
    {
        return io.getProcessorPivotPosition();
    }

    public void setPosition(double setPoint)
    {
        io.setPosition(setPoint);
        setpointVal = setPoint;
    }
}
