package frc.robot.subsystems.processor_pivot;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.util.SparkUtil;

public class IO_ProcessorPivotSim implements IO_ProcessorPivotBase {

  private final Alert configAlert =
      new Alert("Processor pivot için config ayarlanırken bir hata oluştu.", AlertType.kError);

  DCMotor maxGearbox = DCMotor.getNEO(1);

  SparkMax sparkMax =
      new SparkMax(ProcessorPivotConstants.kProcessorPivotMotorID, MotorType.kBrushless);

  SparkMaxConfig config = new SparkMaxConfig();
  SparkMaxSim maxSim = new SparkMaxSim(sparkMax, maxGearbox);
  SparkClosedLoopController m_controller;
  private SparkAbsoluteEncoderSim processorAbsoluteEncoder;

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          maxGearbox,
          ProcessorPivotConstants.kGearing,
          SingleJointedArmSim.estimateMOI(
              ProcessorPivotConstants.kArmLength, ProcessorPivotConstants.kMass),
          ProcessorPivotConstants.kArmLength,
          Units.rotationsToRadians(ProcessorPivotConstants.kProcessorPivotMinAngle),
          Units.rotationsToRadians(ProcessorPivotConstants.kProcessorPivotMaxAngle),
          true,
          0);

  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  public IO_ProcessorPivotSim() {
    SparkUtil.tryUntilOk(
        sparkMax,
        5,
        () ->
            sparkMax.configure(
                config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(ProcessorPivotConstants.kIsInverted),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
        configAlert);

    processorAbsoluteEncoder = maxSim.getAbsoluteEncoderSim();
    processorAbsoluteEncoder.setPosition(0);
    m_controller = sparkMax.getClosedLoopController();
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));
    FaultReporter.getInstance()
        .registerHardware(
            ProcessorPivotConstants.kSubsystemName, "Processor Pivot Motor", sparkMax);
  }

  @Override
  public void updateInputs(ProcessorPivotInputs inputs) {
    m_armSim.setInput(maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_armSim.update(0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

    inputs.processorPivotAppliedVolts = maxSim.getAppliedOutput() * maxSim.getBusVoltage();
    inputs.processorPivotCurrentAmps = maxSim.getMotorCurrent();
    inputs.processorPivotPositionRads = Units.rotationsToRadians(getProcessorPivotPosition());
  }

  @Override
  public void setSpeed(double speed) {
    maxSim.setAppliedOutput(speed);
  }

  @Override
  public void setPosition(double setPoint) {
    m_controller.setReference(setPoint, ControlType.kPosition);
  }

  @Override
  public void setVoltage(Voltage volts) {
    maxSim.setBusVoltage(volts.magnitude());
  }

  @Override
  public void stopMotor() {
    maxSim.setAppliedOutput(0);
  }

  @Override
  public double getProcessorPivotPosition() {
    return processorAbsoluteEncoder.getPosition();
  }
}
