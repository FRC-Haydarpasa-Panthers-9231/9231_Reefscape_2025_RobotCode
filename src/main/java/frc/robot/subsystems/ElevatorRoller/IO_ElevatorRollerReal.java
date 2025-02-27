package frc.robot.subsystems.ElevatorRoller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.Constants;
import frc.robot.util.BeamBreak;
import frc.robot.util.SparkUtil;

public class IO_ElevatorRollerReal implements IO_ElevatorRollerBase {

  private SparkMax elevatorRoller1Motor;
  private SparkMax elevatorRoller2Motor;
  private final BeamBreak photoelectricSensor = new BeamBreak(Constants.kBeamBreakPort);
  private final Alert configAlert =
      new Alert("Elevator Roller için config ayarlanırken bir hata oluştu.", AlertType.kError);

  public IO_ElevatorRollerReal() {
    elevatorRoller1Motor =
        new SparkMax(ElevatorRollerConstants.kElevatorRollerMotor1Port, MotorType.kBrushless);
    elevatorRoller2Motor =
        new SparkMax(ElevatorRollerConstants.kElevatorRollerMotor2Port, MotorType.kBrushless);

    SparkUtil.tryUntilOk(
        elevatorRoller1Motor,
        5,
        () ->
            elevatorRoller1Motor.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        configAlert);

    SparkUtil.tryUntilOk(
        elevatorRoller2Motor,
        5,
        () ->
            elevatorRoller2Motor.configure(
                new SparkMaxConfig()
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .follow(elevatorRoller1Motor, true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        configAlert);

    FaultReporter.getInstance()
        .registerHardware(
            ElevatorRollerConstants.kSubsystemName,
            "Elevator Roller Motor 1",
            elevatorRoller1Motor);
    FaultReporter.getInstance()
        .registerHardware(
            ElevatorRollerConstants.kSubsystemName,
            "Elevator Roller Motor 2",
            elevatorRoller2Motor);
  }

  @Override
  public boolean hasCoral() {
    return photoelectricSensor.isTrue();
  }

  @Override
  public void updateInputs(ElevatorRollerInputs inputs) {
    inputs.elevatorRoller1AppliedVolts =
        elevatorRoller1Motor.getAppliedOutput() * elevatorRoller1Motor.getBusVoltage();
    inputs.elevatorRoller1CurrentAmps = elevatorRoller1Motor.getOutputCurrent();

    inputs.elevatorRoller2AppliedVolts =
        elevatorRoller2Motor.getAppliedOutput() * elevatorRoller2Motor.getBusVoltage();
    inputs.elevatorRoller2CurrentAmps = elevatorRoller2Motor.getOutputCurrent();
    inputs.elevatorRoller1TempCelsius = elevatorRoller1Motor.getMotorTemperature();
    inputs.elevatorRoller1TempCelsius = elevatorRoller2Motor.getMotorTemperature();
    // inputs.hasCoral = photoelectricSensor.get();
  }

  @Override
  public void setElevatorRollerSpeed(double speed) {
    elevatorRoller1Motor.set(speed);
    elevatorRoller2Motor.set(speed);
  }

  @Override
  public void stopMotors() {
    elevatorRoller1Motor.stopMotor();
    elevatorRoller2Motor.stopMotor();
  }
}
