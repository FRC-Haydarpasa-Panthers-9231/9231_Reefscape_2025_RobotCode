package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class IO_ElevatorReal implements IO_ElevatorBase {
  public static final double reduction = Constants.Elevator.kElevatorGearing;
  private final TalonFX elevatorMotor;
  private final TalonFX followerElevatorMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public IO_ElevatorReal() {
    elevatorMotor = new TalonFX(0);
    followerElevatorMotor = new TalonFX(1);

    followerElevatorMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0).withKV(0).withKS(0).withKA(0);
    config.Feedback.SensorToMechanismRatio = reduction;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    config.MotionMagic.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    // config.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    PhoenixUtil.tryUntilOk(5, () -> elevatorMotor.getConfigurator().apply(config));
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
    if (!isPositionWithinLimits(elevatorMotor.getPosition().getValueAsDouble())) {
      elevatorMotor.stopMotor();
      return;
    }
    elevatorMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stopMotor() {
    elevatorMotor.stopMotor();
    followerElevatorMotor.stopMotor();
  }

  @Override
  public void runPosition(double positionRad) {
    if (positionRad <= Constants.ELEVATOR_MAX_POSITION_RAD
        && positionRad >= Constants.ELEVATOR_MIN_POSITION_RAD) {
      elevatorMotor.setControl(new MotionMagicVoltage(Units.radiansToRotations(positionRad)));
    }
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
    elevatorMotor.getConfigurator().apply(config);
  }

  public boolean isPositionWithinLimits(double positionRad) {
    return positionRad >= Constants.ELEVATOR_MIN_POSITION_RAD
        && positionRad <= Constants.ELEVATOR_MAX_POSITION_RAD;
  }
}
