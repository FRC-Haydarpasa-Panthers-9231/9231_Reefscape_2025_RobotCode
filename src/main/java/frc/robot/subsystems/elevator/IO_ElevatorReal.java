package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
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
  private StatusSignal<Angle> elevatorPositionStatusSignalFollower;

  private StatusSignal<Temperature> elevatorLeadTempStatusSignal;
  private StatusSignal<Temperature> elevatorFollowerTempStatusSignal;

  private StatusSignal<AngularVelocity> leadVelocitySignal;
  private StatusSignal<AngularVelocity> followerVelocitySignal;

  private StatusSignal<Boolean> elevatorForwardSoftLimitTriggeredSignal;
  private StatusSignal<Boolean> elevatorReverseSoftLimitTriggeredSignal;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final MotionMagicExpoVoltage motionMagicExpoControl = new MotionMagicExpoVoltage(0);

  private final TalonFXConfiguration followerConfig = new TalonFXConfiguration();
  private final Alert configAlert =
      new Alert("Elevator için config ayarlanırken bir hata oluştu.", AlertType.kError);

  private Alert refreshAlert = new Alert("Failed to refresh all signals.", AlertType.kError);

  public IO_ElevatorReal() {
    elevatorMotorLead = new TalonFX(ElevatorConstants.kElevatorMotorLeadID, "Drivetrain");
    elevatorMotorFollower = new TalonFX(ElevatorConstants.kElevatorMotorFollowerID, "Drivetrain");

    leadStatorCurrent = elevatorMotorLead.getStatorCurrent();
    followerStatorCurrent = elevatorMotorFollower.getStatorCurrent();

    leadVoltageSupplied = elevatorMotorLead.getMotorVoltage();
    followerVoltageSupplied = elevatorMotorFollower.getMotorVoltage();

    leadSupplyCurrent = elevatorMotorLead.getSupplyCurrent();
    followerSupplyCurrent = elevatorMotorFollower.getSupplyCurrent();

    elevatorPositionStatusSignal = elevatorMotorLead.getPosition();
    elevatorPositionStatusSignalFollower = elevatorMotorFollower.getPosition();

    elevatorLeadTempStatusSignal = elevatorMotorLead.getDeviceTemp();
    elevatorFollowerTempStatusSignal = elevatorMotorFollower.getDeviceTemp();

    leadVelocitySignal = elevatorMotorLead.getRotorVelocity();
    followerVelocitySignal = elevatorMotorFollower.getRotorVelocity();

    elevatorForwardSoftLimitTriggeredSignal = elevatorMotorLead.getFault_ForwardSoftLimit();
    elevatorReverseSoftLimitTriggeredSignal = elevatorMotorLead.getFault_ReverseSoftLimit();

    PhoenixUtil.tryUntilOk(
        5,
        () -> elevatorMotorLead.getConfigurator().apply(ElevatorConstants.kElavatorConfig),
        configAlert);

    followerConfig.Feedback.SensorToMechanismRatio = 12;
    followerConfig.Voltage.PeakForwardVoltage = 12.0;
    followerConfig.Voltage.PeakReverseVoltage = -12;
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followerConfig
        .CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(34));

    PhoenixUtil.tryUntilOk(
        5, () -> elevatorMotorFollower.getConfigurator().apply(followerConfig), configAlert);

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
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
        elevatorPositionStatusSignalFollower,
        followerVelocitySignal);

    // Optimize CAN bus usage for all devices
    elevatorMotorLead.optimizeBusUtilization(4, 0.1);
    elevatorMotorFollower.optimizeBusUtilization(4, 0.1);

    FaultReporter.getInstance()
        .registerHardware(
            ElevatorConstants.kSubsystemName, "Elevator Motor Lead", elevatorMotorLead);

    FaultReporter.getInstance()
        .registerHardware(
            ElevatorConstants.kSubsystemName, "Elevator Motor Follower", elevatorMotorLead);
    setSensorPosition(0);
    elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {

    StatusCode status =
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
            elevatorPositionStatusSignalFollower,
            leadVelocitySignal,
            followerVelocitySignal,
            elevatorForwardSoftLimitTriggeredSignal,
            elevatorReverseSoftLimitTriggeredSignal);
    PhoenixUtil.checkError(status, "Failed to refresh elevator motor signals.", refreshAlert);
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
    inputs.positionRadsFollower =
        Units.rotationsToRadians(elevatorPositionStatusSignalFollower.getValueAsDouble());
    inputs.velocityLead = leadVelocitySignal.getValueAsDouble();
    inputs.velocityFollower = followerVelocitySignal.getValueAsDouble();

    inputs.elevatorForwardSoftLimitTriggered = elevatorForwardSoftLimitTriggeredSignal.getValue();
    inputs.elevatorReverseSoftLimitTriggered = elevatorReverseSoftLimitTriggeredSignal.getValue();

    /*
     * LoggedTunableNumber.ifChanged(
     * hashCode(),
     * motionMagic -> {
     * TalonFXConfiguration config = new TalonFXConfiguration();
     * this.elevatorMotorLead.getConfigurator().refresh(config);
     *
     * config.Slot0.kP = motionMagic[0];
     * config.Slot0.kI = motionMagic[1];
     * config.Slot0.kD = motionMagic[2];
     * config.Slot0.kS = motionMagic[3];
     * config.Slot0.kV = motionMagic[4];
     * config.Slot0.kA = motionMagic[5];
     * config.Slot0.kG = motionMagic[6];
     * config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[7];
     * config.MotionMagic.MotionMagicAcceleration = motionMagic[8];
     * config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[9];
     * config.MotionMagic.MotionMagicJerk = motionMagic[9];
     * config.MotionMagic.MotionMagicExpo_kV = motionMagic[9];
     *
     * PhoenixUtil.tryUntilOk(
     * 5, () -> elevatorMotorLead.getConfigurator().apply(config), configAlert);
     * },
     * kPslot0,
     * kIslot0,
     * kDslot0,
     * kSslot0,
     * kVslot0,
     * kAslot0,
     * kGslot0,
     * cruiseVelocity,
     * acceleration,
     * motionMagicJerk,
     * expo_kv);
     */
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
  public void setPosition(double goalPositionRads) {

    elevatorMotorLead.setControl(
        motionMagicExpoControl
            .withPosition(Units.radiansToRotations(goalPositionRads))
            .withSlot(0));
    elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
  }

  /*
   *
   *
   * elevatorMotorLead.setControl(
   * motionMagicPositionRequest
   * .withPosition(Units.radiansToRotations(goalPositionRads))
   * .withSlot(0));
   * elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
   */

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
    return Units.rotationsToRadians(elevatorMotorLead.getPosition().getValueAsDouble());
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    return elevatorMotorLead.getRotorVelocity().getValue();
  }

  @Override
  public boolean isRotorVelocityZero() {
    return getRotorVelocity().isNear(edu.wpi.first.units.Units.RotationsPerSecond.zero(), 0.01);
  }

  public boolean getForwardSoftLimitTriggered() {
    return elevatorForwardSoftLimitTriggeredSignal.getValue();
  }

  public boolean getReverseSoftLimitTriggered() {
    return elevatorReverseSoftLimitTriggeredSignal.getValue();
  }
}
