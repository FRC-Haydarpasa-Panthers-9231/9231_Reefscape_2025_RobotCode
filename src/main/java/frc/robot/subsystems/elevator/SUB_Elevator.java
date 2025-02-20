package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SUB_Elevator extends SubsystemBase {

  private double lastDesiredPosition;

  public boolean attemptingZeroing = false;
  public boolean hasZeroed = false;

  private static SysIdRoutine sysIdRoutine;

  private final IO_ElevatorBase io;
  public final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  private static final LoggedTunableNumber elevatorHeightSetpoint =
      new LoggedTunableNumber("Elevator/Elevator Height Setpoint", 0);
  private static final LoggedTunableNumber elevatorDebugVolts =
      new LoggedTunableNumber("Elevator/Elevator Volts", 2);

  public SUB_Elevator(IO_ElevatorBase io) {
    this.io = io;

    lastDesiredPosition = 0;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setElevatorVoltage(volts), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("isElevatorAtSetpoint", this.isAtSetPoint());
  }

  /**
   * Asansör motorlarının voltajını ayarlar
   *
   * @param volts Çalıştırmak istediginiz volt degeri.
   */
  public void setElevatorVoltage(Voltage volts) {
    io.setElevatorVoltage(volts);
  }

  public void setElevatorDebugVoltage() {
    io.setElevatorVoltage(Volts.of(elevatorDebugVolts.getAsDouble()));
  }

  /**
   * Kraken encoderlarını belirli bir degere atamak için kullanılır. Kullanım alanı genelde
   * encoder'ı sıfırlamak ve default pozisyon atamaktır.
   *
   * @param setpoint enconder'ın ayarlanacagı deger
   */
  public void setEncoderPosition(double setpoint) {
    io.setSensorPosition(setpoint);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command sysIDCharacterizationRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> SignalLogger.start()),
        sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> getPosition() == ElevatorConstants.kForwardLimit),
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> getPosition() == ElevatorConstants.kReverseLimit),
        sysIdDynamic(SysIdRoutine.Direction.kForward)
            .until(() -> getPosition() == ElevatorConstants.kForwardLimit),
        sysIdDynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> getPosition() == ElevatorConstants.kReverseLimit),
        Commands.runOnce(() -> SignalLogger.stop()));
  }

  public void setSpeed(double speed) {
    io.setElevatorSpeed(speed);
  }

  /** Asansör motorlarını durdurur. */
  public void stopElevator() {
    io.stopMotor();
  }

  public double getLastDesiredPosition() {
    return lastDesiredPosition;
  }

  public boolean isAtSetPoint() {
    double currentPosition = io.getElevatorPosition(); // Elevator pozisyonunu double olarak al
    double tolerance = ElevatorConstants.kTolerance; // Tolerans değeri (double olarak
    // varsayıyoruz)
    double targetPosition = getLastDesiredPosition(); // Hedef pozisyon

    return (currentPosition > targetPosition - tolerance)
        && (currentPosition < targetPosition + tolerance);
  }

  public void setCoastMode(Boolean coastMode) {
    io.setCoastMode(coastMode);
  }

  public void setNeutral() {
    io.setNeutral();
  }

  public void setPosition(double positionRad) {
    io.setPosition(positionRad);
  }

  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    io.setSoftwareLimits(reverseLimitEnable, forwardLimitEnable);
  }

  public void setPositionDebug() {
    io.setPosition(elevatorHeightSetpoint.getAsDouble());
  }

  public AngularVelocity getRotorVelocity() {
    return io.getRotorVelocity();
  }

  public double getPosition() {
    return io.getElevatorPosition();
  }

  public boolean isRotorVelocityZero() {
    return io.isRotorVelocityZero();
  }
}
