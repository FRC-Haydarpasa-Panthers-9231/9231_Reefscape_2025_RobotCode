package frc.robot.subsystems.processor_roller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class IO_ProcessorRollerReal implements IO_ProcessorRollerBase {

  private SparkMax processorRoller;

  public IO_ProcessorRollerReal() {
    processorRoller =
        new SparkMax(Constants.ElevatorRoller.kElevatorRollerMotor1Port, MotorType.kBrushless);

    SparkUtil.tryUntilOk(
        processorRoller,
        5,
        () ->
            processorRoller.configure(
                new SparkMaxConfig()
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(50)
                    .inverted(false),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        super.getClass().getName());
  }

  @Override
  public void updateInputs(ProcessorRollerInputs inputs) {
    inputs.processorRollerAppliedVolts =
        processorRoller.getAppliedOutput() * processorRoller.getBusVoltage();
    inputs.processorRollerCurrentAmps = processorRoller.getOutputCurrent();
  }

  @Override
  public void setProcessorRollerSpeed(double speed) {
    processorRoller.setVoltage(MathUtil.clamp(12 * speed, -12, 12));
  }

  @Override
  public void stopMotor() {
    processorRoller.stopMotor();
  }
}
