package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.sim.MotionProfiledElevatorMechanism;
import frc.robot.util.sim.MotionProfiledMechanism;
import org.littletonrobotics.junction.Logger;

public class IO_ElevatorSim implements IO_ElevatorBase {

  private final TalonFX elevatorMotor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public static ElevatorSim elevatorSim;
  private final MotionProfiledMechanism m_Mech;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final LoggedTunableNumber elevatorPosition =
      new LoggedTunableNumber("Elevator/Position", 0);
  private final LoggedTunableNumber elevatorPositionInside =
      new LoggedTunableNumber("Elevator/PositionInside", 0);
  private double lastElevatorHeight = 0;
  private double elevatorOutHeight = 0.146;

  public IO_ElevatorSim() {
    elevatorMotor = new TalonFX(0);

    config.Slot0 =
        new Slot0Configs().withKP(50).withKI(5).withKD(1).withKV(0).withKS(0).withKA(0).withKG(0.1);
    config.Feedback.SensorToMechanismRatio = Constants.Elevator.kElevatorGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity =
        80 / Constants.Elevator.kElevatorGearing; // Target
    // cruise
    // velocity
    // of
    // 80
    // rps
    config.MotionMagic.MotionMagicAcceleration =
        160 / Constants.Elevator.kElevatorGearing; // Target acceleration of 160 rps/s (0.5
    // seconds)
    config.MotionMagic.MotionMagicJerk = 1600 / Constants.Elevator.kElevatorGearing; // Target
    // jerk of
    // 1600
    // rps/s/s
    // (0.1
    // seconds)

    PhoenixUtil.tryUntilOk(5, () -> elevatorMotor.getConfigurator().apply(config));

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
    // Get the simulation state for the lead motor
    var simState = elevatorMotor.getSimState();

    // set the supply (battery) voltage for the lead motor simulation state
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = elevatorMotor.getMotorVoltage().getValueAsDouble();

    elevatorSim.setInputVoltage(motorVoltage);
    elevatorSim.update(0.020);

    // meters / 2πr = drum rotations, drum rotations * gear ratio = rotor rotations
    simState.setRawRotorPosition(
        elevatorSim.getPositionMeters()
            / (2 * Math.PI * Constants.Elevator.kElevatorDrumRadius)
            * Constants.Elevator.kElevatorGearing);
    simState.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond()
            / (2 * Math.PI * Constants.Elevator.kElevatorDrumRadius)
            * Constants.Elevator.kElevatorGearing);

    m_Mech.updateElevator(elevatorSim.getPositionMeters());
    /*
     * if (elevatorPosition.hasChanged(elevatorPosition.hashCode())
     * || elevatorPositionInside.hasChanged(elevatorPositionInside.hashCode())) {
     * Logger.recordOutput(
     * "FinalComponentPoses2",
     * new Pose3d[] {
     * new Pose3d(0.1, 0.006, 0.182 + elevatorPosition.get(),
     * new Rotation3d(0, 0, 0))
     * });
     *
     * Logger.recordOutput(
     * "FinalComponentPoses3",
     * new Pose3d[] {
     * new Pose3d(0.32, 0.01, 0.505 + elevatorPosition.get(),
     * new Rotation3d(0, 0, 0))
     * });
     *
     * Logger.recordOutput(
     * "FinalComponentPoses1",
     * new Pose3d[] {
     * new Pose3d(0.07, 0.01, 0.146 + elevatorPositionInside.get(),
     * new Rotation3d(0, 0, 0))
     * });
     * }
     */
    if (elevatorSim.getPositionMeters() > lastElevatorHeight
        && 1.8 + elevatorOutHeight > 0.182 + elevatorSim.getPositionMeters()) {

      elevatorOutHeight = 0.146 + elevatorSim.getPositionMeters();
      Logger.recordOutput(
          "FinalComponentPoses1",
          new Pose3d[] {new Pose3d(0.07, 0.01, elevatorOutHeight, new Rotation3d(0, 0, 0))});

      Logger.recordOutput(
          "FinalComponentPoses2",
          new Pose3d[] {
            new Pose3d(0.1, 0.006, 0.182 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
          });

      Logger.recordOutput(
          "FinalComponentPoses3",
          new Pose3d[] {
            new Pose3d(0.32, 0.01, 0.505 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
          });
    } else if (elevatorSim.getPositionMeters() < lastElevatorHeight
        && elevatorOutHeight <= 0.182 + elevatorSim.getPositionMeters()) {
      elevatorOutHeight = 0.146 + elevatorSim.getPositionMeters();

      Logger.recordOutput(
          "FinalComponentPoses1",
          new Pose3d[] {new Pose3d(0.07, 0.01, elevatorOutHeight, new Rotation3d(0, 0, 0))});

      Logger.recordOutput(
          "FinalComponentPoses2",
          new Pose3d[] {
            new Pose3d(0.1, 0.006, 0.182 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
          });

      Logger.recordOutput(
          "FinalComponentPoses3",
          new Pose3d[] {
            new Pose3d(0.32, 0.01, 0.505 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
          });

    } else if (elevatorSim.getPositionMeters() == lastElevatorHeight) {
      Logger.recordOutput(
          "FinalComponentPoses1",
          new Pose3d[] {
            new Pose3d(0.07, 0.01, 0.146 + elevatorOutHeight, new Rotation3d(0, 0, 0))
          });

      Logger.recordOutput(
          "FinalComponentPoses2",
          new Pose3d[] {
            new Pose3d(0.1, 0.006, 0.182 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
          });

      Logger.recordOutput(
          "FinalComponentPoses3",
          new Pose3d[] {
            new Pose3d(0.32, 0.01, 0.505 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
          });
    } else if (elevatorOutHeight >= 0.182 + elevatorSim.getPositionMeters()
        || 1.986 + elevatorOutHeight < 0.182 + elevatorSim.getPositionMeters()
            && (elevatorSim.getPositionMeters() < lastElevatorHeight
                || elevatorSim.getPositionMeters() > lastElevatorHeight)) {

      Logger.recordOutput(
          "FinalComponentPoses2",
          new Pose3d[] {
            new Pose3d(0.1, 0.006, 0.182 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
          });

      Logger.recordOutput(
          "FinalComponentPoses3",
          new Pose3d[] {
            new Pose3d(0.32, 0.01, 0.505 + elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0))
          });
    }
    lastElevatorHeight = elevatorSim.getPositionMeters();

    /*
     * // if (0.182 + elevatorPosition.get() <= 0.637)
     * lastElevatorHeight = elevatorSim.getPositionMeters();
     *
     * // asansör yükseliyorsa
     * if (lastElevatorHeight < elevatorSim.getPositionMeters()) {
     *
     * if (0.146 + elevatorSim.getPositionMeters())
     *
     * Logger.recordOutput(
     * "FinalComponentPoses2",
     * new Pose3d[] {
     * new Pose3d(
     * 0.1,
     * 0.006,
     * 0.182 + elevatorSim.getPositionMeters(),
     * new Rotation3d(0, 0, 0))
     * });
     *
     *
     * Logger.recordOutput(
     * "FinalComponentPoses3",
     * new Pose3d[] {
     * new Pose3d(0.32, 0.01, 0.505 + elevatorSim.getPositionMeters(),
     * new Rotation3d(0, 0, 0))
     * });
     *
     * if (0.182 + elevatorSim.getPositionMeters() >= 0.637) {
     *
     * Logger.recordOutput(
     * "FinalComponentPoses1",
     * new Pose3d[] {new Pose3d(0.07, 0.01, 0.146 + elevatorSim.getPositionMeters(),
     * new Rotation3d(0, 0, 0))});
     *
     * }
     * }
     *
     * // asansör alçalıyorsa
     * if (lastElevatorHeight > elevatorSim.getPositionMeters()) {
     *
     *
     *
     * }
     */

    // 0.78
    /*
     * if(0.182 + elevatorPosition.get() >= 0.637 || (0.182 + elevatorPosition.get()<=0.637
     * &&
     * 0.146 + elevatorPosition.get()) ){
     * Logger.recordOutput(
     * "FinalComponentPoses3",
     * new Pose3d[] {
     * new Pose3d(0.07, 0.01, 0.146 + elevatorPosition.get(), new Rotation3d(0, 0, 0))
     * });
     * }
     */

    /*
     * z
     * Logger.recordOutput(
     * "FinalComponentPoses2",
     * new Pose3d[] {
     * new Pose3d(0.1, 0.006, 0.182 + elevatorSim.getPositionMeters(),
     * new Rotation3d(0, 0, 0))
     * });
     * Logger.recordOutput(
     * "FinalComponentPoses3",
     * new Pose3d[] {
     * new Pose3d(0.07, 0.01, 0.146 + elevatorSim.getPositionMeters(),
     * new Rotation3d(0, 0, 0))
     * });
     */

    inputs.elevatorAppliedVolts =
        new double[] {elevatorMotor.getMotorVoltage().getValueAsDouble(), 0.0};

    inputs.elevatorPositionRad =
        Units.rotationsToRadians(elevatorMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setElevatorVoltage(double volts) {
    /*
     * if(!isPositionWithinLimits(elevatorMotor.getPosition().getValueAsDouble())){
     * elevatorMotor.stopMotor(); return; }
     */
    elevatorMotor.setControl(voltageRequest.withOutput(MathUtil.clamp(volts, -12, 12)));
  }

  @Override
  public void stopMotor() {
    elevatorMotor.stopMotor();
  }

  @Override
  public void runPosition(double positionRad) {
    // if(positionRad <= Constants.ELEVATOR_MAX_POSITION_RAD &&
    // positionRad >= Constants.ELEVATOR_MIN_POSITION_RAD){
    elevatorMotor.setControl(new MotionMagicVoltage(Units.radiansToRotations(positionRad)));
    // }
  }

  @Override
  public boolean isPositionWithinLimits(double positionRad) {
    // positionRad >= Constants.ELEVATOR_MIN_POSITION_RAD && positionRad <=
    // Constants.ELEVATOR_MAX_POSITION_RAD;
    return true;
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
}
