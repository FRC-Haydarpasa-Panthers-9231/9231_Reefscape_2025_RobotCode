package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.PantherUtil;
import frc.robot.util.PhoenixUtil;

public class IO_ElevatorReal implements IO_ElevatorBase {
  private final TalonFX elevatorMotor;
  private final TalonFX followerElevatorMotor;

  private final TalonFXConfiguration configLeader = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public IO_ElevatorReal() {
    elevatorMotor = new TalonFX(Constants.Elevator.kElevatorMotor1Port);
    followerElevatorMotor = new TalonFX(Constants.Elevator.kElevatorMotor2Port);

    followerElevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), true));

    configLeader.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configLeader.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configLeader.Slot0 =
        new Slot0Configs().withKP(0).withKI(0).withKD(0).withKV(0).withKS(0).withKA(0);

    configLeader.Feedback.SensorToMechanismRatio = Constants.Elevator.kElevatorGearing;

    configLeader.CurrentLimits.StatorCurrentLimit = 80.0;
    configLeader.CurrentLimits.StatorCurrentLimitEnable = true;
    configLeader
        .CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(50));
    configLeader
        .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(
            PantherUtil.metersToRotations(Constants.Elevator.kMaxElevatorHeightMeters))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(
            PantherUtil.metersToRotations(Constants.Elevator.kMinElevatorHeightMeters));
    configLeader.Voltage.PeakForwardVoltage = 12.0;
    configLeader.Voltage.PeakReverseVoltage = -12;
    // configLeader.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configLeader.MotionMagic.MotionMagicCruiseVelocity = 80; // Target

    configLeader.MotionMagic.MotionMagicAcceleration = 160; // Target

    configLeader.MotionMagic.MotionMagicJerk = 1600; // Target

    PhoenixUtil.tryUntilOk(
        5, () -> elevatorMotor.getConfigurator().apply(configLeader), super.getClass().getName());
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.elevatorAppliedVolts =
        new double[] {
          elevatorMotor.getMotorVoltage().getValueAsDouble(),
          followerElevatorMotor.getMotorVoltage().getValueAsDouble()
        };
    inputs.elevatorCurrentAmps =
        new double[] {
          elevatorMotor.getStatorCurrent().getValueAsDouble(),
          followerElevatorMotor.getStatorCurrent().getValueAsDouble()
        };
    inputs.elevatorTempCelcius =
        new double[] {
          elevatorMotor.getDeviceTemp().getValueAsDouble(),
          followerElevatorMotor.getDeviceTemp().getValueAsDouble()
        };
    inputs.elevatorPositionRad =
        Units.rotationsToRadians(elevatorMotor.getPosition().getValueAsDouble());
    inputs.elevatorVelocityRadPerSec =
        Units.rotationsToRadians(elevatorMotor.getVelocity().getValueAsDouble());
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
    elevatorMotor.stopMotor();
    followerElevatorMotor.stopMotor();
  }

  @Override
  public void runPosition(double positionRad) {

    elevatorMotor.setControl(new MotionMagicVoltage(Units.radiansToRotations(positionRad)));
  }

  @Override
  public void setSlot0(
      double kG, double kS, double kV, double kA, double kP, double kI, double kD) {
    configLeader.Slot0.kG = kG;
    configLeader.Slot0.kS = kS;
    configLeader.Slot0.kV = kV;
    configLeader.Slot0.kA = kA;
    configLeader.Slot0.kP = kP;
    configLeader.Slot0.kI = kI;
    configLeader.Slot0.kD = kD;
    PhoenixUtil.tryUntilOk(
        5, () -> elevatorMotor.getConfigurator().apply(configLeader), super.getClass().getName());
  }
}
