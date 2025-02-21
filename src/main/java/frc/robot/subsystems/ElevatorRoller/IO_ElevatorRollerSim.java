package frc.robot.subsystems.ElevatorRoller;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.Constants;
import frc.robot.subsystems.processor_roller.ProcessorRollerConstants;
import frc.robot.util.SparkUtil;

public class IO_ElevatorRollerSim implements IO_ElevatorRollerBase {
  private final DigitalInput photoelectricSensor = new DigitalInput(Constants.kBeamBreakPort);

  private final Alert configAlert =
      new Alert("Elevator Roller için config ayarlanırken bir hata oluştu.", AlertType.kError);

  DCMotor maxGearbox = DCMotor.getNEO(1);

  SparkMax elevatorRollerMotor1 =
      new SparkMax(ElevatorRollerConstants.kElevatorRollerMotor1Port, MotorType.kBrushless);
  SparkMax elevatorRollerMotor2 =
      new SparkMax(ElevatorRollerConstants.kElevatorRollerMotor2Port, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  SparkMaxSim elevatorRollerMotor1Sim = new SparkMaxSim(elevatorRollerMotor1, maxGearbox);
  SparkMaxSim elevatorRollerMotor2Sim = new SparkMaxSim(elevatorRollerMotor2, maxGearbox);

  public IO_ElevatorRollerSim() {

    SparkUtil.tryUntilOk(
        elevatorRollerMotor1,
        5,
        () ->
            elevatorRollerMotor1.configure(
                config.idleMode(IdleMode.kBrake).smartCurrentLimit(50),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        configAlert);
    SparkUtil.tryUntilOk(
        elevatorRollerMotor2,
        5,
        () ->
            elevatorRollerMotor2.configure(
                config.idleMode(IdleMode.kBrake).smartCurrentLimit(50),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        configAlert);

    FaultReporter.getInstance()
        .registerHardware(
            ProcessorRollerConstants.kSubsystemName,
            "Elevator Roller Motor 1",
            elevatorRollerMotor1);
    FaultReporter.getInstance()
        .registerHardware(
            ProcessorRollerConstants.kSubsystemName,
            "Elevator Roller Motor 2",
            elevatorRollerMotor2);
  }

  @Override
  public void updateInputs(ElevatorRollerInputs inputs) {
    inputs.elevatorRoller1AppliedVolts =
        elevatorRollerMotor1Sim.getAppliedOutput() * elevatorRollerMotor1Sim.getBusVoltage();
    inputs.elevatorRoller2AppliedVolts =
        elevatorRollerMotor2Sim.getAppliedOutput() * elevatorRollerMotor2Sim.getBusVoltage();
    inputs.elevatorRoller1CurrentAmps = elevatorRollerMotor1Sim.getMotorCurrent();
    inputs.elevatorRoller2CurrentAmps = elevatorRollerMotor2Sim.getMotorCurrent();
  }

  @Override
  public void setElevatorRollerSpeed(double speed) {

    elevatorRollerMotor1Sim.setAppliedOutput(speed);
    elevatorRollerMotor2Sim.setAppliedOutput(speed);
  }

  @Override
  public void stopMotors() {
    elevatorRollerMotor1Sim.setAppliedOutput(0);
    elevatorRollerMotor2Sim.setAppliedOutput(0);
  }

  @Override
  public boolean hasCoral() {
    return photoelectricSensor.get();
  }
}
