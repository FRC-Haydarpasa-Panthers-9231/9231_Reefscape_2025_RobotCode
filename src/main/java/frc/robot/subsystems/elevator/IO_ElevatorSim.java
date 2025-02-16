package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.elevator.ElevatorConstants.kElevatorTeeth;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.PantherUtil;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.sim.MotionProfiledElevatorMechanism;
import frc.robot.util.sim.MotionProfiledMechanism;
import org.littletonrobotics.junction.Logger;

public class IO_ElevatorSim implements IO_ElevatorBase {

  private final TalonFX elevatorMotorLead;
  private final TalonFX elevatorMotorFollower;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public static ElevatorSim elevatorSim;
  private final MotionProfiledMechanism m_Mech;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final MotionMagicVoltage motionMagicPositionRequest = new MotionMagicVoltage(0.0);

  private StatusSignal<Current> leadStatorCurrent;
  private StatusSignal<Current> followerStatorCurrent;

  private StatusSignal<Voltage> leadVoltageSupplied;
  private StatusSignal<Voltage> followerVoltageSupplied;

  private StatusSignal<Current> leadSupplyCurrent;
  private StatusSignal<Current> followerSupplyCurrent;

  private StatusSignal<Angle> elevatorPositionStatusSignal;

  private StatusSignal<Temperature> elevatorLeadTempStatusSignal;
  private StatusSignal<Temperature> elevatorFollowerTempStatusSignal;

  public IO_ElevatorSim() {
    elevatorMotorLead = new TalonFX(ElevatorConstants.kElevatorMotorLeadID);
    elevatorMotorFollower = new TalonFX(ElevatorConstants.kElevatorMotorFollowerID);

    elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));

    leadStatorCurrent = elevatorMotorLead.getStatorCurrent();
    followerStatorCurrent = elevatorMotorFollower.getStatorCurrent();

    leadVoltageSupplied = elevatorMotorLead.getMotorVoltage();
    followerVoltageSupplied = elevatorMotorFollower.getMotorVoltage();

    leadSupplyCurrent = elevatorMotorLead.getSupplyCurrent();
    followerSupplyCurrent = elevatorMotorFollower.getSupplyCurrent();

    elevatorPositionStatusSignal = elevatorMotorLead.getPosition();

    elevatorLeadTempStatusSignal = elevatorMotorLead.getDeviceTemp();
    elevatorFollowerTempStatusSignal = elevatorMotorFollower.getDeviceTemp();

    config.Slot0 =
        new Slot0Configs().withKP(50).withKI(5).withKD(1).withKV(0).withKS(0).withKA(0).withKG(0.1);

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));
    /**
     * config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true) .withForwardSoftLimitThreshold(
     * PantherUtil.metersToRotations(ElevatorConstants.kMaxElevatorHeightMeters))
     * .withReverseSoftLimitEnable(true) .withReverseSoftLimitThreshold(
     * PantherUtil.metersToRotations(ElevatorConstants.kMinElevatorHeightMeters));
     */
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12;

    config.MotionMagic.MotionMagicCruiseVelocity = 80; // Target
    // cruise
    // velocity
    // of
    // 80
    // rps
    config.MotionMagic.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5
    // seconds)
    config.MotionMagic.MotionMagicJerk = 1600; // Target
    // jerk of
    // 1600
    // rps/s/s
    // (0.1
    // seconds)

    PhoenixUtil.tryUntilOk(
        5, () -> elevatorMotorLead.getConfigurator().apply(config), super.getClass().getName());

    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            ElevatorConstants.kElevatorGearing,
            ElevatorConstants.kCarriageMass,
            ElevatorConstants.kElevatorDrumRadius,
            ElevatorConstants.kMinElevatorHeightMeters.magnitude(),
            ElevatorConstants.kMaxElevatorHeightMeters.magnitude(),
            false,
            ElevatorConstants.kDefaultSetpoint);

    m_Mech = new MotionProfiledElevatorMechanism("Elevator");
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevatorPositionStatusSignal,
        leadStatorCurrent,
        followerStatorCurrent,
        leadSupplyCurrent,
        followerSupplyCurrent,
        leadVoltageSupplied,
        followerVoltageSupplied,
        elevatorLeadTempStatusSignal,
        elevatorFollowerTempStatusSignal);

    inputs.voltageSuppliedLead = leadVoltageSupplied.getValueAsDouble();
    inputs.voltageSuppliedFollower = followerVoltageSupplied.getValueAsDouble();

    inputs.statorCurrentAmpsLead = leadStatorCurrent.getValueAsDouble();
    inputs.statorCurrentAmpsFollower = followerStatorCurrent.getValueAsDouble();

    inputs.supplyCurrentAmpsLead = leadSupplyCurrent.getValueAsDouble();
    inputs.supplyCurrentAmpsFollower = followerSupplyCurrent.getValueAsDouble();

    inputs.leadTempCelsius = elevatorLeadTempStatusSignal.getValueAsDouble();
    inputs.followerTempCelsius = elevatorFollowerTempStatusSignal.getValueAsDouble();

    inputs.closedLoopError = elevatorMotorLead.getClosedLoopError().getValueAsDouble();

    inputs.closedLoopReference = elevatorMotorLead.getClosedLoopReference().getValueAsDouble();

    inputs.positionRotations = elevatorPositionStatusSignal.getValueAsDouble();

    inputs.positionMeters =
        PantherUtil.rotationsToMeters(elevatorPositionStatusSignal.getValueAsDouble());

    // Ana motorun simulasyon durumunu al
    var simState = elevatorMotorLead.getSimState();

    // set the supply (battery) voltage for the lead motor simulation state
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = elevatorMotorLead.getMotorVoltage().getValueAsDouble();

    elevatorSim.setInputVoltage(motorVoltage);
    elevatorSim.update(0.020);

    simState.setRawRotorPosition(
        elevatorSim.getPositionMeters() / (2 * ElevatorConstants.kElevatorPitch * kElevatorTeeth));
    simState.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond()
            / (2 * ElevatorConstants.kElevatorPitch * kElevatorTeeth));

    m_Mech.updateElevator(elevatorSim.getPositionMeters());

    Logger.recordOutput(
        "FinalComponentPoses1",
        new Pose3d[] {
          new Pose3d(
              0.07, 0.01, 0.146 + elevatorSim.getPositionMeters() / 2, new Rotation3d(0, 0, 0))
        });

    Logger.recordOutput(
        "FinalComponentPoses2",
        new Pose3d[] {
          new Pose3d(0.1, 0.006, 0.178 + (elevatorSim.getPositionMeters()), new Rotation3d(0, 0, 0))
        });

    Logger.recordOutput(
        "FinalComponentPoses3",
        new Pose3d[] {
          new Pose3d(0.32, 0.01, 0.501 + (elevatorSim.getPositionMeters()), new Rotation3d(0, 0, 0))
        });
  }

  @Override
  public void setElevatorVoltage(double volts) {
    elevatorMotorLead.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setElevatorSpeed(double speed) {
    elevatorMotorLead.set(speed);
  }

  @Override
  public void stopMotor() {
    elevatorMotorFollower.stopMotor();
    elevatorMotorLead.stopMotor();
  }

  @Override
  public void runPositionRads(double positionRad) {
    elevatorMotorLead.setControl(
        motionMagicPositionRequest.withPosition(Units.radiansToRotations(positionRad)));
  }

  @Override
  public void runPositionMeters(double positionMeters) {
    elevatorMotorLead.setControl(
        motionMagicPositionRequest.withPosition(
            PantherUtil.metersToRotations(Units.radiansToRotations(positionMeters))));
  }
}
