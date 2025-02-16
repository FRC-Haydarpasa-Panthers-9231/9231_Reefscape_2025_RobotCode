package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PantherUtil;
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
      new LoggedTunableNumber("Elevator/Cruise Velocity", 80);

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

    configElevatorMotorLead(elevatorMotorLead);
    configElevatorMotorFollower(elevatorMotorFollower);

    elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
  }

  private void configElevatorMotorLead(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    MotionMagicConfigs leadMotorConfig = config.MotionMagic;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /*
     *
     * SoftwareLimitSwitchConfigs softLimitConfigs =
     * new SoftwareLimitSwitchConfigs()
     * .withForwardSoftLimitEnable(true)
     * .withForwardSoftLimitThreshold(ElevatorConstants.kForwardSoftLimitThreshold)
     * .withReverseSoftLimitEnable(true)
     * .withReverseSoftLimitThreshold(ElevatorConstants.kReverseSoftLimitThreshold);
     *
     */

    // config.SoftwareLimitSwitch = softLimitConfigs;

    config.Slot0.kP = kPslot0.get();
    config.Slot0.kI = kIslot0.get();
    config.Slot0.kD = kDslot0.get();
    config.Slot0.kS = kSslot0.get();
    config.Slot0.kV = kVslot0.get();
    config.Slot0.kA = kAslot0.get();
    config.Slot0.kG = kGslot0.get();

    config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

    config.Slot1.withGravityType(GravityTypeValue.Elevator_Static);

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));

    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12;
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // config.MotionMagic.MotionMagicCruiseVelocity = 80; // Target

    config.MotionMagic.MotionMagicAcceleration = 20; // Target

    config.MotionMagic.MotionMagicJerk = 0; // Target

    leadMotorConfig.MotionMagicCruiseVelocity = cruiseVelocity.get();

    PhoenixUtil.tryUntilOk(
        5, () -> elevatorMotorLead.getConfigurator().apply(config), super.getClass().getName());

    FaultReporter.getInstance()
        .registerHardware(ElevatorConstants.kSubsystemName, "Elevator Motor Lead", motor);
  }

  public void configElevatorMotorFollower(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(
        5, () -> elevatorMotorFollower.getConfigurator().apply(config), super.getClass().getName());

    FaultReporter.getInstance()
        .registerHardware(ElevatorConstants.kSubsystemName, "Elevator Motor Follower", motor);
  }

  @Override
  public void zeroPosition() {
    elevatorMotorLead.setPosition(0.0);
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
        cruiseVelocity);
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
    elevatorMotorLead.stopMotor();
    elevatorMotorFollower.stopMotor();
  }

  @Override
  public void runPositionMeters(double meters) {

    elevatorMotorLead.setControl(
        new MotionMagicVoltage(PantherUtil.metersToRotations(meters)).withSlot(0));
  }

  @Override
  public void runPositionRads(double positionRad) {
    elevatorMotorLead.setControl(
        motionMagicPositionRequest.withPosition(Units.radiansToRotations(positionRad)));
  }
}
