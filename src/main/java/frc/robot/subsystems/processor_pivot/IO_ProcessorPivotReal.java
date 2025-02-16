package frc.robot.subsystems.processor_pivot;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;

public class IO_ProcessorPivotReal implements IO_ProcessorPivotBase {

  SparkMax processorPivot;
  SparkClosedLoopController m_controller;

  private SparkAbsoluteEncoder processorAbsoluteEncoder;
  SparkMaxConfig config = new SparkMaxConfig();

  public IO_ProcessorPivotReal() {
    processorPivot =
        new SparkMax(ProcessorPivotConstants.kProcessorPivotMotorID, MotorType.kBrushless);

    SparkUtil.tryUntilOk(
        processorPivot,
        5,
        () ->
            processorPivot.configure(
                config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(ProcessorPivotConstants.kIsInverted),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        super.getClass().getName());

    config
        .closedLoop
        .p(ProcessorPivotConstants.kP)
        .i(ProcessorPivotConstants.kI)
        .d(ProcessorPivotConstants.kD)
        .minOutput(ProcessorPivotConstants.kProcessorPivotMinOutput)
        .maxOutput(ProcessorPivotConstants.kProcessorPivotMaxOutput);
    config
        .softLimit
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(ProcessorPivotConstants.kProcessorPivotMaxAngle)
        .reverseSoftLimit(ProcessorPivotConstants.kProcessorPivotMinAngle);

    config
        .idleMode(ProcessorPivotConstants.kBrake ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(50)
        .inverted(ProcessorPivotConstants.kIsInverted);

    SparkUtil.tryUntilOk(
        processorPivot,
        5,
        () ->
            processorPivot.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters),
        super.getClass().getName());

    processorPivot.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_controller = processorPivot.getClosedLoopController();
  }

  @Override
  public void setSpeed(double speed) {
    processorPivot.set(speed);
  }

  @Override
  public void stopMotor() {
    processorPivot.stopMotor();
  }

  @Override
  public double getProcessorPivotPosition() {
    return processorAbsoluteEncoder.getPosition();
  }

  @Override
  public void setPosition(double setPoint) {
    m_controller.setReference(setPoint, ControlType.kPosition);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.closedLoop.p(kP).i(kI).d(kD);
  }

  @Override
  public void updateInputs(ProcessorPivotInputs inputs) {
    inputs.processorPivotAppliedVolts =
        processorPivot.getAppliedOutput() * processorPivot.getBusVoltage();
    inputs.processorPivotCurrentAmps = processorPivot.getOutputCurrent();
    inputs.elevatorPositionRad = Units.rotationsToRadians(getProcessorPivotPosition());
  }
}
