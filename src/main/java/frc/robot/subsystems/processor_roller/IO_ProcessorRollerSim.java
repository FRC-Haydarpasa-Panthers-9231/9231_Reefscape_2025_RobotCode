package frc.robot.subsystems.processor_roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IO_ProcessorRollerSim implements IO_ProcessorRollerBase {

  private DCMotorSim processorRoller1Motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.00032, 1),
          DCMotor.getNeoVortex(1));

  private double processorRoller1MotorAppliedVolts = 0.0;

  public IO_ProcessorRollerSim() {}

  @Override
  public void updateInputs(ProcessorRollerInputs inputs) {
    inputs.processorRollerAppliedVolts = processorRoller1MotorAppliedVolts;
    inputs.processorRollerCurrentAmps = processorRoller1Motor.getCurrentDrawAmps();
  }

  @Override
  public void setProcessorRollerVoltage(double speed) {
    processorRoller1MotorAppliedVolts = MathUtil.clamp(12 * speed, -12, 12);
    processorRoller1Motor.setInputVoltage(processorRoller1MotorAppliedVolts);
  }

  @Override
  public void stopMotor() {
    processorRoller1MotorAppliedVolts = 0;

    processorRoller1Motor.setInputVoltage(processorRoller1MotorAppliedVolts);
  }
}
