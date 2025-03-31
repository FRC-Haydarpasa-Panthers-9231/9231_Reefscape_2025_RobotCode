package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.sensors.CoralSensorIOBeam;
import org.littletonrobotics.junction.Logger;

public class SUB_ElevatorRoller extends SubsystemBase {

  private final IO_ElevatorRollerBase io;

  public final ElevatorRollerInputsAutoLogged rollerInputs = new ElevatorRollerInputsAutoLogged();
  private final CoralSensorIOInputsAutoLogged coralSensorInputs =
      new CoralSensorIOInputsAutoLogged();

  private static final LoggedTunableNumber elevatorRollerDebugSpeed =
      new LoggedTunableNumber("ElevatorRoller/Elevator Roller speed", 0.3);
  private CoralSensorIOBeam coralSensor;

  public SUB_ElevatorRoller(IO_ElevatorRollerBase io, CoralSensorIOBeam coralSensor) {
    this.io = io;
    this.coralSensor = coralSensor;
  }

  @Override
  public void periodic() {
    io.updateInputs(rollerInputs);
    Logger.processInputs("ElevatorRoller/Rollers", rollerInputs);
    coralSensorIO.updateInputs(coralSensorInputs);
    Logger.processInputs("ElevatorRoller/CoralSensor", coralSensorInputs);
  }

  public void setSpeed(double speed) {
    io.setElevatorRollerSpeed(speed);
  }

  public void setDebugSpeed(boolean isForward) {
    double speed =
        isForward
            ? 1 * elevatorRollerDebugSpeed.getAsDouble()
            : -1 * elevatorRollerDebugSpeed.getAsDouble();
    io.setElevatorRollerSpeed(speed);
  }

  public boolean hasCoral() {
    return coralSensor.getValue();
  }

  public Command stopmotors() {
    return Commands.runOnce(io::stopMotors, this);
  }

  public Command scoreCoralwithJustRollers() {
    return Commands.runOnce(
            () -> io.setElevatorRollerSpeed(ElevatorRollerConstants.kScoringSpeed), this)
        .until(() -> this.hasCoral())
        .finallyDo(
            () -> {
              Commands.waitSeconds(
                  ElevatorRollerConstants.kCoralScoreTime.in(edu.wpi.first.units.Units.Seconds));
              Commands.runOnce(io::stopMotors, this);
            });
  }

  public Command intakingCoral() {
    return Commands.runOnce(
            () -> io.setElevatorRollerSpeed(ElevatorRollerConstants.kIntakingSpeed), this)
        .until(() -> this.hasCoral())
        .finallyDo(io::stopMotors);
  }
}
