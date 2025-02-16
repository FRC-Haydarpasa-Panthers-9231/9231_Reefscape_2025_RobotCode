package frc.robot.subsystems.ElevatorRoller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class IO_ElevatorRollerReal implements IO_ElevatorRollerBase {

  private SparkMax elevatorRoller1Motor;
  private SparkMax elevatorRoller2Motor;

  public IO_ElevatorRollerReal() {
    elevatorRoller1Motor =
        new SparkMax(Constants.ElevatorRoller.kElevatorRollerMotor1Port, MotorType.kBrushless);
    elevatorRoller2Motor =
        new SparkMax(Constants.ElevatorRoller.kElevatorRollerMotor2Port, MotorType.kBrushless);

    SparkUtil.tryUntilOk(
        elevatorRoller1Motor,
        5,
        () ->
            elevatorRoller1Motor.configure(
                new SparkMaxConfig()
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(50)
                    .inverted(false),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        super.getClass().getName());

    SparkUtil.tryUntilOk(
        elevatorRoller2Motor,
        5,
        () ->
            elevatorRoller2Motor.configure(
                new SparkMaxConfig()
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(50)
                    .inverted(true)
                    .follow(elevatorRoller1Motor),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        super.getClass().getName());
  }

  @Override
  public void updateInputs(ElevatorRollerInputs inputs) {
    inputs.elevatorRoller1AppliedVolts =
        elevatorRoller1Motor.getAppliedOutput() * elevatorRoller1Motor.getBusVoltage();
    inputs.elevatorRoller1CurrentAmps = elevatorRoller1Motor.getOutputCurrent();

    inputs.elevatorRoller2AppliedVolts =
        elevatorRoller2Motor.getAppliedOutput() * elevatorRoller2Motor.getBusVoltage();
    inputs.elevatorRoller2CurrentAmps = elevatorRoller2Motor.getOutputCurrent();
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
