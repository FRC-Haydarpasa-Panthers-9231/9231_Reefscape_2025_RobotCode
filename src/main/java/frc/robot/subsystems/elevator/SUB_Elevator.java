package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SUB_Elevator extends SubsystemBase {

    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0);
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);

    private static SysIdRoutine sysIdRoutine;

    private final IO_ElevatorBase io;
    public final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    public SUB_Elevator(IO_ElevatorBase io)
    {
        this.io = io;

        sysIdRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                    (volts) -> io.setElevatorVoltage(volts.in(Volts)), null, this));
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        /*
         * if(!io.isPositionWithinLimits(inputs.elevatorPositionRad)) { io.stopMotor(); }
         */

        // Update tunable numbers
        if (kG.hasChanged(hashCode())
            || kS.hasChanged(hashCode())
            || kV.hasChanged(hashCode())
            || kA.hasChanged(hashCode())
            || kP.hasChanged(hashCode())
            || kI.hasChanged(hashCode())
            || kD.hasChanged(hashCode())) {
            io.setSlot0(kG.get(), kS.get(), kV.get(), kA.get(), kP.get(), kI.get(), kD.get());
        }
    }

    @Override
    public void simulationPeriodic()
    {}

    /**
     * Asansör motorlarının voltajını ayarlar
     * 
     * @param volts Çalıştırmak istediğiniz volt değeri.
     */
    public void setElevatorVoltage(double volts)
    {
        io.setElevatorVoltage(volts);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Asansörü belli bir pozisyona götürür.
     * 
     * @param positionRad başlangıç konumundan motorun radyan cinsinden sabitlenmek istendiği konum.
     */
    public void runPosition(double positionRad)
    {
        io.runPosition(positionRad);
    }

    /**
     * Asansör motorlarını durdurur.
     */
    public void stopElevator()
    {
        io.stopMotor();
    }
}
