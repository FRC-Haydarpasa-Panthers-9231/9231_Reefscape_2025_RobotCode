package frc.robot.subsystems.ElevatorRoller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IO_ElevatorRollerSim implements IO_ElevatorRollerBase {
  private final DigitalInput photoelectricSensor = new DigitalInput(Constants.kBeamBreakPort);
  private DCMotorSim elevatorRoller1Motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.00032, 1),
          DCMotor.getNeoVortex(1));

  private DCMotorSim elevatorRoller2Motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.00032, 1),
          DCMotor.getNeoVortex(1));
  private double elevatorRoller1MotorAppliedVolts = 0.0;
  private double elevatorRoller2MotorAppliedVolts = 0.0;

  public IO_ElevatorRollerSim() {}

  @Override
  public void updateInputs(ElevatorRollerInputs inputs) {

    inputs.elevatorRoller1AppliedVolts = elevatorRoller1MotorAppliedVolts;
    inputs.elevatorRoller1CurrentAmps = elevatorRoller1Motor.getCurrentDrawAmps();

    inputs.elevatorRoller1AppliedVolts = elevatorRoller2MotorAppliedVolts;
    inputs.elevatorRoller2CurrentAmps = elevatorRoller2Motor.getCurrentDrawAmps();
  }

  @Override
  public void setElevatorRollerSpeed(double speed) {
    elevatorRoller1MotorAppliedVolts = MathUtil.clamp(12 * speed, -12, 12);
    elevatorRoller2MotorAppliedVolts = MathUtil.clamp(12 * speed, -12, 12);

    elevatorRoller1Motor.setInputVoltage(elevatorRoller1MotorAppliedVolts);
    elevatorRoller2Motor.setInputVoltage(elevatorRoller2MotorAppliedVolts);
  }

  @Override
  public void stopMotors() {
    elevatorRoller1MotorAppliedVolts = 0;
    elevatorRoller2MotorAppliedVolts = 0;
    elevatorRoller1Motor.setInputVoltage(elevatorRoller1MotorAppliedVolts);
    elevatorRoller2Motor.setInputVoltage(elevatorRoller2MotorAppliedVolts);
  }

  @Override
  public boolean hasCoral() {
    return photoelectricSensor.get();
  }
}
