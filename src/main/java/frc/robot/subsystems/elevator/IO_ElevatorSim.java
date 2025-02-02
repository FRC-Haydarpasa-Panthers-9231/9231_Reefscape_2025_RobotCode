package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.sim.MotionProfiledElevatorMechanism;
import frc.robot.util.sim.MotionProfiledMechanism;

public class IO_ElevatorSim implements IO_ElevatorBase {

  private final TalonFX elevatorMotor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public static ElevatorSim elevatorSim;
  private final MotionProfiledMechanism m_Mech;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public IO_ElevatorSim() {
    elevatorMotor = new TalonFX(0);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 =
        new Slot0Configs()
            .withKP(300)
            .withKI(50)
            .withKD(50)
            .withKV(0)
            .withKS(0)
            .withKA(0)
            .withKG(0.1);
    config.Feedback.SensorToMechanismRatio = Constants.Elevator.kElevatorGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    config.MotionMagic.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    config.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    PhoenixUtil.tryUntilOk(5, () -> elevatorMotor.getConfigurator().apply(config));

    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            Constants.Elevator.kElevatorGearing,
            Constants.Elevator.kCarriageMass,
            Constants.Elevator.kElevatorDrumRadius,
            Constants.Elevator.kMinElevatorHeightMeters,
            Constants.Elevator.kMaxElevatorHeightMeters,
            true,
            0.24);

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
