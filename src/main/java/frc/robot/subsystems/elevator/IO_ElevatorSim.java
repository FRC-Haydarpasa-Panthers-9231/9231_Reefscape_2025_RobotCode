package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.elevator.ElevatorConstants.MOTION_MAGIC_ACCELERATION;
import static frc.robot.subsystems.elevator.ElevatorConstants.MOTION_MAGIC_EXPO_KV;
import static frc.robot.subsystems.elevator.ElevatorConstants.kElevatorGearing;
import static frc.robot.subsystems.elevator.ElevatorConstants.kElevatorTeeth;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.sim.MotionProfiledElevatorMechanism;
import frc.robot.util.sim.MotionProfiledMechanism;
import org.littletonrobotics.junction.Logger;

public class IO_ElevatorSim implements IO_ElevatorBase {

    private final TalonFX elevatorMotorLead;
    private final TalonFX elevatorMotorFollower;

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

    private final LoggedTunableNumber kPslot0 =
        new LoggedTunableNumber("Elevator/kPslot0", ElevatorConstants.KP_SLOT1);
    private final LoggedTunableNumber kIslot0 =
        new LoggedTunableNumber("Elevator/kIslot0", ElevatorConstants.KI_SLOT1);
    private final LoggedTunableNumber kDslot0 =
        new LoggedTunableNumber("Elevator/kDslot0", ElevatorConstants.KD_SLOT1);
    private final LoggedTunableNumber kSslot0 =
        new LoggedTunableNumber("Elevator/kSslot0", ElevatorConstants.KS_SLOT1);
    private final LoggedTunableNumber kVslot0 =
        new LoggedTunableNumber("Elevator/kVslot0", ElevatorConstants.KV_SLOT1);
    private final LoggedTunableNumber kAslot0 =
        new LoggedTunableNumber("Elevator/kAslot0", ElevatorConstants.KA_SLOT1);
    private final LoggedTunableNumber kGslot0 =
        new LoggedTunableNumber("Elevator/kGslot0", ElevatorConstants.KG_SLOT1);

    private final LoggedTunableNumber cruiseVelocity =
        new LoggedTunableNumber(
            "Elevator/Cruise Velocity", ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY);

    private final LoggedTunableNumber acceleration =
        new LoggedTunableNumber("Elevator/acceleration", MOTION_MAGIC_ACCELERATION);

    private final LoggedTunableNumber expoKv =
        new LoggedTunableNumber("Elevator/expo kV", MOTION_MAGIC_EXPO_KV);

    public IO_ElevatorSim()
    {
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
    public void updateInputs(ElevatorInputs inputs)
    {
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

        inputs.tempCelciusLead = elevatorLeadTempStatusSignal.getValueAsDouble();
        inputs.tempCelciusFollower = elevatorFollowerTempStatusSignal.getValueAsDouble();

        inputs.closedLoopError = elevatorMotorLead.getClosedLoopError().getValueAsDouble();

        inputs.closedLoopReference = elevatorMotorLead.getClosedLoopReference().getValueAsDouble();

        inputs.positionRotations = elevatorPositionStatusSignal.getValueAsDouble();
        inputs.positionMeters =
            edu.wpi.first.units.Units.Meters.of(elevatorPositionStatusSignal.getValueAsDouble());
        inputs.positionRads =
            Units.rotationsToRadians(elevatorPositionStatusSignal.getValueAsDouble());

        // Ana motorun simulasyon durumunu al
        var simState = elevatorMotorLead.getSimState();

        // set the supply (battery) voltage for the lead motor simulation state
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var motorVoltage = elevatorMotorLead.getMotorVoltage().getValueAsDouble();

        elevatorSim.setInputVoltage(motorVoltage);
        elevatorSim.update(Constants.loopPeriodSecs);

        simState.setRawRotorPosition(
            elevatorSim.getPositionMeters()
                / (2 * ElevatorConstants.kElevatorPitch * kElevatorTeeth)
                * kElevatorGearing);
        simState.setRotorVelocity(
            elevatorSim.getVelocityMetersPerSecond()
                / (2 * ElevatorConstants.kElevatorPitch * kElevatorTeeth)
                * kElevatorGearing);

        m_Mech.updateElevator(elevatorSim.getPositionMeters());

        Logger.recordOutput(
            "FinalComponentPoses1",
            new Pose3d[] {
                    new Pose3d(
                        0.07, 0.01, 0.146 + elevatorSim.getPositionMeters() / 2,
                        new Rotation3d(0, 0, 0))
            });

        Logger.recordOutput(
            "FinalComponentPoses2",
            new Pose3d[] {
                    new Pose3d(0.1, 0.006, 0.178 + (elevatorSim.getPositionMeters()),
                        new Rotation3d(0, 0, 0))
            });

        Logger.recordOutput(
            "FinalComponentPoses3",
            new Pose3d[] {
                    new Pose3d(0.32, 0.01, 0.501 + (elevatorSim.getPositionMeters()),
                        new Rotation3d(0, 0, 0))
            });

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
            cruiseVelocity,
            acceleration,
            expoKv);
    }

    @Override
    public void setElevatorVoltage(Voltage volts)
    {
        elevatorMotorLead.setControl(voltageRequest.withOutput(volts));
        elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
    }

    @Override
    public void setElevatorSpeed(double speed)
    {
        elevatorMotorLead.set(speed);
    }

    @Override
    public void stopMotor()
    {
        elevatorMotorFollower.stopMotor();
        elevatorMotorLead.stopMotor();
    }



    @Override
    public void setSensorPosition(Distance setpoint)
    {
        elevatorMotorLead.setPosition(setpoint.in(Meters));
        elevatorMotorFollower.setPosition(setpoint.in(Meters));
    }

    @Override
    public void setCoastMode(Boolean coastMode)
    {
        if (coastMode) {
            elevatorMotorLead.getConfigurator().apply(ElevatorConstants.kCoastModeConfiguration);
            elevatorMotorFollower.getConfigurator()
                .apply(ElevatorConstants.kCoastModeConfiguration);
        } else {
            elevatorMotorLead.getConfigurator().apply(ElevatorConstants.kElavatorConfig);
            elevatorMotorFollower.getConfigurator().apply(ElevatorConstants.kElavatorConfig);
        }
    }

    @Override
    public void setNeutral()
    {
        elevatorMotorLead.setControl(new NeutralOut());
        elevatorMotorFollower.setControl(new NeutralOut());
    }

    @Override
    public void setPosition(Distance height)
    {
        elevatorMotorLead.setControl(motionMagicPositionRequest.withPosition(height.in(Meters)));
        elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
    }

    @Override
    public void runPositionRads(double positionRad)
    {
        elevatorMotorLead.setControl(
            motionMagicPositionRequest.withPosition(Units.radiansToRotations(positionRad)));
        elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));

    }

    @Override
    public Distance getElevatorPosition()
    {
        return edu.wpi.first.units.Units.Meters
            .of(elevatorMotorLead.getPosition().getValueAsDouble());
    }

    @Override
    public AngularVelocity getRotorVelocity()
    {
        return elevatorMotorLead.getRotorVelocity().getValue();
    }

    @Override
    public boolean isRotorVelocityZero()
    {
        return getRotorVelocity().isNear(edu.wpi.first.units.Units.RotationsPerSecond.zero(), 0.01);
    }

    @Override
    public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable)
    {
        ElevatorConstants.kElavatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable =
            reverseLimitEnable;
        ElevatorConstants.kElavatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable =
            forwardLimitEnable;

        elevatorMotorLead.getConfigurator().apply(ElevatorConstants.kElavatorConfig);
        elevatorMotorFollower.getConfigurator().apply(ElevatorConstants.kElavatorConfig);
    }
}
