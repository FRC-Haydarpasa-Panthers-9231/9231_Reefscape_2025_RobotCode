package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

public class IO_ElevatorReal implements IO_ElevatorBase {
  private TalonFX elevatorMotorLead;
  private TalonFX elevatorMotorFollower;

  private StatusSignal<Current> leadStatorCurrent;
  private StatusSignal<Current> followerStatorCurrent;

  private StatusSignal<Voltage> leadVoltageSupplied;
  private StatusSignal<Voltage> followerVoltageSupplied;

  private StatusSignal<Current> leadSupplyCurrent;
  private StatusSignal<Current> followerSupplyCurrent;

  private StatusSignal<Angle> elevatorPositionStatusSignal;

  private StatusSignal<Temperature> elevatorLeadTempStatusSignal;
  private StatusSignal<Temperature> elevatorFollowerTempStatusSignal;

  private StatusSignal<AngularVelocity> leadVelocitySignal;
  private StatusSignal<AngularVelocity> followerVelocitySignal;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final MotionMagicVoltage motionMagicPositionRequest = new MotionMagicVoltage(0.0);

  private final LoggedTunableNumber kPslot0 =
      new LoggedTunableNumber("Elevator/kPslot0", ElevatorConstants.KP_SLOT0);
  private final LoggedTunableNumber kIslot0 =
      new LoggedTunableNumber("Elevator/kIslot0", ElevatorConstants.KI_SLOT0);
  private final LoggedTunableNumber kDslot0 =
      new LoggedTunableNumber("Elevator/kDslot0", ElevatorConstants.KD_SLOT0);
  private final LoggedTunableNumber kSslot0 =
      new LoggedTunableNumber("Elevator/kSslot0", ElevatorConstants.KS_SLOT0);
  private final LoggedTunableNumber kVslot0 =
      new LoggedTunableNumber("Elevator/kVslot0", ElevatorConstants.KV_SLOT0);
  private final LoggedTunableNumber kAslot0 =
      new LoggedTunableNumber("Elevator/kAslot0", ElevatorConstants.KA_SLOT0);
  private final LoggedTunableNumber kGslot0 =
      new LoggedTunableNumber("Elevator/kGslot0", ElevatorConstants.KG_SLOT0);

  private final LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber(
          "Elevator/Cruise Velocity", ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY);

  private final LoggedTunableNumber acceleration =
      new LoggedTunableNumber("Elevator/acceleration", ElevatorConstants.MOTION_MAGIC_ACCELERATION);

  private final LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber("Elevator/expo_kv", ElevatorConstants.MOTION_MAGIC_JERK);

  private final LoggedTunableNumber expo_kv =
      new LoggedTunableNumber("Elevator/expo_kv", ElevatorConstants.MOTION_MAGIC_KV);

  public IO_ElevatorReal() {
    elevatorMotorLead = new TalonFX(ElevatorConstants.kElevatorMotorLeadID);
    elevatorMotorFollower = new TalonFX(ElevatorConstants.kElevatorMotorFollowerID);

    leadStatorCurrent = elevatorMotorLead.getStatorCurrent();
    followerStatorCurrent = elevatorMotorFollower.getStatorCurrent();

    leadVoltageSupplied = elevatorMotorLead.getMotorVoltage();
    followerVoltageSupplied = elevatorMotorFollower.getMotorVoltage();

    leadSupplyCurrent = elevatorMotorLead.getSupplyCurrent();
    followerSupplyCurrent = elevatorMotorFollower.getSupplyCurrent();

    elevatorPositionStatusSignal = elevatorMotorLead.getPosition();

    elevatorLeadTempStatusSignal = elevatorMotorLead.getDeviceTemp();
    elevatorFollowerTempStatusSignal = elevatorMotorFollower.getDeviceTemp();
    leadVelocitySignal = elevatorMotorLead.getRotorVelocity();
    followerVelocitySignal = elevatorMotorFollower.getRotorVelocity();

    PhoenixUtil.tryUntilOk(
        5,
        () -> elevatorMotorLead.getConfigurator().apply(ElevatorConstants.kElavatorConfig),
        super.getClass().getName());
    PhoenixUtil.tryUntilOk(
        5,
        () -> elevatorMotorFollower.getConfigurator().apply(ElevatorConstants.kElavatorConfig),
        super.getClass().getName());

    FaultReporter.getInstance()
        .registerHardware(
            ElevatorConstants.kSubsystemName, "Elevator Motor Lead", elevatorMotorLead);

    FaultReporter.getInstance()
        .registerHardware(
            ElevatorConstants.kSubsystemName, "Elevator Motor Follower", elevatorMotorLead);
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
        elevatorFollowerTempStatusSignal,
        leadVelocitySignal,
        followerVelocitySignal);

    inputs.voltageSuppliedLead = leadVoltageSupplied.getValueAsDouble();
    inputs.voltageSuppliedFollower = followerVoltageSupplied.getValueAsDouble();

    inputs.statorCurrentAmpsLead = leadStatorCurrent.getValueAsDouble();
    inputs.statorCurrentAmpsFollower = followerStatorCurrent.getValueAsDouble();

    inputs.supplyCurrentAmpsLead = leadSupplyCurrent.getValueAsDouble();
    inputs.supplyCurrentAmpsFollower = followerSupplyCurrent.getValueAsDouble();

    inputs.tempCelciusLead = elevatorLeadTempStatusSignal.getValueAsDouble();
    inputs.tempCelciusFollower = elevatorFollowerTempStatusSignal.getValueAsDouble();

    inputs.closedLoopError = elevatorMotorLead.getClosedLoopError().getValueAsDouble();

    inputs.closedLoopReference = elevatorMotorLead.getClosedLoopReference().getValueAsDouble();

    inputs.positionRotations = elevatorPositionStatusSignal.getValueAsDouble();

    inputs.positionRads = Units.rotationsToRadians(elevatorPositionStatusSignal.getValueAsDouble());
    inputs.velocityLead = leadVelocitySignal.getValueAsDouble();
    inputs.velocityFollower = followerVelocitySignal.getValueAsDouble();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.elevatorMotorLead.getConfigurator().refresh(config);

          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kG = motionMagic[6];
          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[7];
          config.MotionMagic.MotionMagicAcceleration = motionMagic[8];
          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[9];
          config.MotionMagic.MotionMagicJerk = motionMagic[9];
          config.MotionMagic.MotionMagicExpo_kV = motionMagic[9];

          PhoenixUtil.tryUntilOk(
              5,
              () -> elevatorMotorLead.getConfigurator().apply(config),
              super.getClass().getName());
        },
        kPslot0,
        kIslot0,
        kDslot0,
        kSslot0,
        kVslot0,
        kAslot0,
        kGslot0,
        cruiseVelocity,
        acceleration,
        motionMagicJerk,
        expo_kv);
  }

  @Override
  public void setCoastMode(Boolean coastMode) {
    if (coastMode) {
      elevatorMotorLead.getConfigurator().apply(ElevatorConstants.kCoastModeConfiguration);
      elevatorMotorFollower.getConfigurator().apply(ElevatorConstants.kCoastModeConfiguration);
    } else {
      elevatorMotorLead.getConfigurator().apply(ElevatorConstants.kElavatorConfig);
      elevatorMotorFollower.getConfigurator().apply(ElevatorConstants.kElavatorConfig);
    }
  }

  @Override
  public void setSensorPosition(double setpoint) {
    elevatorMotorLead.setPosition(setpoint);
    elevatorMotorFollower.setPosition(setpoint);
  }

  @Override
  public void setNeutral() {
    elevatorMotorLead.setControl(new NeutralOut());
    elevatorMotorFollower.setControl(new NeutralOut());
  }

  @Override
  public void setElevatorVoltage(Voltage volts) {
    elevatorMotorLead.setControl(voltageRequest.withOutput(volts));
    elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
  }

  @Override
  public void setElevatorSpeed(double speed) {
    elevatorMotorLead.set(speed);
  }

  @Override
  public void stopMotor() {
    elevatorMotorLead.stopMotor();
    elevatorMotorFollower.stopMotor();
  }

  @Override
  public void setPosition(double positionRads) {
    elevatorMotorLead.setControl(motionMagicPositionRequest.withPosition(positionRads));
    elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
  }

  @Override
  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    ElevatorConstants.kElavatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable =
        reverseLimitEnable;
    ElevatorConstants.kElavatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        forwardLimitEnable;

    elevatorMotorLead.getConfigurator().apply(ElevatorConstants.kElavatorConfig);
    elevatorMotorFollower.getConfigurator().apply(ElevatorConstants.kElavatorConfig);
  }

  @Override
  public double getElevatorPosition() {
    return elevatorMotorLead.getPosition().getValueAsDouble();
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    return elevatorMotorLead.getRotorVelocity().getValue();
  }

  @Override
  public boolean isRotorVelocityZero() {
    return getRotorVelocity().isNear(edu.wpi.first.units.Units.RotationsPerSecond.zero(), 0.01);
  }
}
