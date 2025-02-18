package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class SUB_Elevator extends SubsystemBase {

    private Distance lastDesiredPosition;

    public boolean attemptingZeroing = false;
    public boolean hasZeroed = false;

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
                    (volts) -> io.setElevatorVoltage(volts), null, this));
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("isElevatorAtSetpoint", this.isAtSetPoint());
    }

    /**
     * Asansör motorlarının voltajını ayarlar
     *
     * @param volts Çalıştırmak istediğiniz volt değeri.
     */
    public void setElevatorVoltage(Voltage volts)
    {
        io.setElevatorVoltage(volts);
    }


    /**
     * Kraken encoderlarını belirli bir değere atamak için kullanılır. Kullanım alanı genelde
     * encoder'ı sıfırlamak ve default pozisyon atamaktır.
     * 
     * @param setpoint enconder'ın ayarlanacağı değer
     */
    public void setEncoderPosition(Distance setpoint)
    {
        io.setSensorPosition(setpoint);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return sysIdRoutine.dynamic(direction);
    }

    public Command sysIDCharacterizationRoutine()
    {
        return Commands.sequence(
            Commands.runOnce(() -> SignalLogger.start()),
            sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> getPosition().magnitude() == ElevatorConstants.kForwardLimit),
            sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                .until(() -> getPosition().magnitude() == ElevatorConstants.kReverseLimit),
            sysIdDynamic(SysIdRoutine.Direction.kForward)
                .until(() -> getPosition().magnitude() == ElevatorConstants.kForwardLimit),
            sysIdDynamic(SysIdRoutine.Direction.kReverse)
                .until(() -> getPosition().magnitude() == ElevatorConstants.kReverseLimit),

            Commands.runOnce(() -> SignalLogger.stop()));
    }

    public void runPositonRads(double positionRads)
    {
        io.runPositionRads(positionRads);
    }

    public void setSpeed(double speed)
    {
        io.setElevatorSpeed(speed);
    }

    /** Asansör motorlarını durdurur. */
    public void stopElevator()
    {
        io.stopMotor();
    }

    public Distance getLastDesiredPosition()
    {
        return lastDesiredPosition;
    }

    public boolean isAtSetPoint()
    {
        return (io.getElevatorPosition()
            .compareTo(getLastDesiredPosition().minus(ElevatorConstants.kTolerance)) > 0)
            && io.getElevatorPosition()
                .compareTo(getLastDesiredPosition().plus(ElevatorConstants.kTolerance)) < 0;
    }

    public void setCoastMode(Boolean coastMode)
    {
        io.setCoastMode(coastMode);
    }


    public void setNeutral()
    {
        io.setNeutral();
    }

    public void setPosition(Distance height)
    {
        io.setPosition(height);
    }

    public void runPositionRads(double positionRad)
    {
        io.runPositionRads(positionRad);
    }

    public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable)
    {
        io.setSoftwareLimits(reverseLimitEnable, forwardLimitEnable);
    }

    public AngularVelocity getRotorVelocity()
    {
        return io.getRotorVelocity();
    }

    public Distance getPosition()
    {
        return io.getElevatorPosition();
    }

    public boolean isRotorVelocityZero()
    {
        return io.isRotorVelocityZero();
    }
}
