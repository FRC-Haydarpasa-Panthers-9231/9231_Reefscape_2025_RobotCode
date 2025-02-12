package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;

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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.PantherUtil;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.sim.MotionProfiledElevatorMechanism;
import frc.robot.util.sim.MotionProfiledMechanism;
import org.littletonrobotics.junction.Logger;

public class IO_ElevatorSim implements IO_ElevatorBase {

  private final TalonFX elevatorMotor;
  private final TalonFX followerElevatorMotor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public static ElevatorSim elevatorSim;
  private final MotionProfiledMechanism m_Mech;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final MotionMagicVoltage motionMagicPositionRequest = new MotionMagicVoltage(0.0);

  public IO_ElevatorSim() {
    elevatorMotor = new TalonFX(Constants.Elevator.kElevatorMotor1Port);
    followerElevatorMotor = new TalonFX(Constants.Elevator.kElevatorMotor2Port);

    followerElevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), true));

    config.Slot0 =
        new Slot0Configs().withKP(50).withKI(5).withKD(1).withKV(0).withKS(0).withKA(0).withKG(0.1);

    config.Feedback.SensorToMechanismRatio = Constants.Elevator.kElevatorGearing;

    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));

    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(
            PantherUtil.metersToRotations(Constants.Elevator.kMaxElevatorHeightMeters))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(
            PantherUtil.metersToRotations(Constants.Elevator.kMinElevatorHeightMeters));

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
        5, () -> elevatorMotor.getConfigurator().apply(config), super.getClass().getName());

    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            Constants.Elevator.kElevatorGearing,
            Constants.Elevator.kCarriageMass,
            Constants.Elevator.kElevatorDrumRadius,
            Constants.Elevator.kMinElevatorHeightMeters,
            Constants.Elevator.kMaxElevatorHeightMeters,
            false,
            Constants.Elevator.kDefaultSetpoint);

    m_Mech = new MotionProfiledElevatorMechanism("Elevator");
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    // Ana motorun simulasyon durumunu al
    var simState = elevatorMotor.getSimState();

    // set the supply (battery) voltage for the lead motor simulation state
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = elevatorMotor.getMotorVoltage().getValueAsDouble();

    elevatorSim.setInputVoltage(motorVoltage);
    elevatorSim.update(0.020);

    simState.setRawRotorPosition(
        elevatorSim.getPositionMeters()
            / (2 * Math.PI * Constants.Elevator.kElevatorDrumRadius)
            * Constants.Elevator.kElevatorGearing);
    simState.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond()
            / (2 * Math.PI * Constants.Elevator.kElevatorDrumRadius)
            * Constants.Elevator.kElevatorGearing);

    m_Mech.updateElevator(elevatorSim.getPositionMeters());

    Logger.recordOutput(
        "FinalComponentPoses1",
        new Pose3d[] {
          new Pose3d(0.07, 0.01, 0.146 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
        });

    Logger.recordOutput(
        "FinalComponentPoses2",
        new Pose3d[] {
          new Pose3d(
              0.1, 0.006, 0.178 + (elevatorSim.getPositionMeters() * 2), new Rotation3d(0, 0, 0))
        });

    Logger.recordOutput(
        "FinalComponentPoses3",
        new Pose3d[] {
          new Pose3d(
              0.32, 0.01, 0.501 + (elevatorSim.getPositionMeters() * 2), new Rotation3d(0, 0, 0))
        });

    inputs.elevatorAppliedVolts =
        new double[] {elevatorMotor.getMotorVoltage().getValueAsDouble(), 0.0};

    inputs.elevatorPositionRad =
        Units.rotationsToRadians(elevatorMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setElevatorVoltage(double volts) {
    elevatorMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  @Override
  public void stopMotor() {
    followerElevatorMotor.stopMotor();
    elevatorMotor.stopMotor();
  }

  @Override
  public void runPosition(double positionRad) {
    elevatorMotor.setControl(
        motionMagicPositionRequest.withPosition(Units.radiansToRotations(positionRad)));
  }

  @Override
  public void setSlot0(
      double kG, double kS, double kV, double kA, double kP, double kI, double kD) {
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    PhoenixUtil.tryUntilOk(
        5, () -> elevatorMotor.getConfigurator().apply(config), super.getClass().getName());
  }
}
