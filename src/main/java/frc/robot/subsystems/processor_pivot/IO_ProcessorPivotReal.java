package frc.robot.subsystems.processor_pivot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class IO_ProcessorPivotReal implements IO_ProcessorPivotBase {

  // FIXME: DOgRU PID DEgERLERİNİ BUL
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Processor_Pivot/kP", ProcessorPivotConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Processor_Pivot/kI", ProcessorPivotConstants.kD);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Processor_Pivot/kD", ProcessorPivotConstants.kI);
  ArmFeedforward pivotFeedForward;

  SparkMax processorPivot;
  // private PIDController processorPivotPID;
  SparkClosedLoopController m_controller;

  SparkMaxConfig config = new SparkMaxConfig();

  private final Alert configAlert =
      new Alert("Processor pivot için config ayarlanırken bir hata oluştu.", AlertType.kError);

  public double degreeAim;

  public IO_ProcessorPivotReal() {
    processorPivot =
        new SparkMax(ProcessorPivotConstants.kProcessorPivotMotorID, MotorType.kBrushless);
    pivotFeedForward = new ArmFeedforward(0, 0.3, 0);
    // processorPivotPID = new PIDController(kP.get(), kI.get(), kD.get());
    m_controller = processorPivot.getClosedLoopController();
    config.closedLoop.p(ProcessorPivotConstants.kP);
    config.closedLoop.i(ProcessorPivotConstants.kI);
    config.closedLoop.d(ProcessorPivotConstants.kD);

    SparkUtil.tryUntilOk(
        processorPivot,
        5,
        () ->
            processorPivot.configure(
                config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(30)
                    .inverted(ProcessorPivotConstants.kIsInverted),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        configAlert);

    FaultReporter.getInstance()
        .registerHardware(
            ProcessorPivotConstants.kSubsystemName, "Processor Pivot Motor", processorPivot);
  }

  /*
   * private void PIDinitialize(double degree)
   * {
   * degreeAim = degree;
   * processorPivotPID.reset();
   * processorPivotPID.setSetpoint(degree);
   * processorPivotPID.setTolerance(0.001);
   * }
   */
  @Override
  public void setSpeed(double speed) {
    processorPivot.set(speed);
  }

  @Override
  public void stopMotor() {
    processorPivot.stopMotor();
  }

  @Override
  public void setVoltage(Voltage volts) {
    processorPivot.setVoltage(volts);
  }

  @Override
  public double getProcessorPivotPosition() {
    return processorPivot.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void setPosition(double setPoint) {

    var ff =
        pivotFeedForward.calculate(Units.degreesToRadians(getProcessorPivotPosition() - 90.2), 0);
    Logger.recordOutput("ProcessorPivot/ff", ff);
    // PIDinitialize(setPoint);
    m_controller.setReference(
        Units.degreesToRotations(setPoint / 3), ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  @Override
  public void updateInputs(ProcessorPivotInputs inputs) {

    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          config.closedLoop.p(values[0]);
          config.closedLoop.i(values[1]);
          config.closedLoop.d(values[2]);

          /*
           * processorPivotPID.setP(values[0]);
           * processorPivotPID.setI(values[1]);
           * processorPivotPID.setD(values[2]);
           */
        },
        kP,
        kI,
        kD);

    inputs.processorPivotAppliedVolts =
        processorPivot.getAppliedOutput() * processorPivot.getBusVoltage();
    inputs.processorPivotCurrentAmps = processorPivot.getOutputCurrent();
    inputs.processorPivotPositionRads = getProcessorPivotPosition();
    inputs.processorPivotTempCelcius = processorPivot.getMotorTemperature();
  }
}
