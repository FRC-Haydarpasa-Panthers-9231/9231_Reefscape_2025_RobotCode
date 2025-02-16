package frc.lib.team3061.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.PantherUtil;
import frc.robot.util.sim.MotionProfiledElevatorMechanism;
import frc.robot.util.sim.MotionProfiledMechanism;
import org.littletonrobotics.junction.Logger;

public class ElevatorSystemSim {

  private TalonFX motor;
  private TalonFXSimState motorSimState;
  private ElevatorSim systemSim;
  private double gearRatio;
  private double pulleyRadiusMeters;
  private String subsystemName;

  private final MotionProfiledMechanism m_Mech = new MotionProfiledElevatorMechanism("Elevator");

  public ElevatorSystemSim(
      TalonFX motor,
      boolean motorInverted,
      double gearRatio,
      double carriageMassKg,
      double pulleyRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double startingHeightMeters,
      String subsystemName) {

    if (Constants.currentMode != Constants.Mode.SIM) {
      return;
    }

    this.motor = motor;
    this.gearRatio = gearRatio;
    this.pulleyRadiusMeters = pulleyRadiusMeters;

    this.motorSimState = this.motor.getSimState();
    this.motorSimState.Orientation =
        motorInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    this.systemSim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            gearRatio,
            carriageMassKg,
            pulleyRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            false,
            startingHeightMeters);

    this.subsystemName = subsystemName;
  }

  public void updateSim() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    this.motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // update the input voltages of the models based on the outputs of the simulated TalonFXs
    double motorVoltage = this.motor.getMotorVoltage().getValueAsDouble();

    this.systemSim.setInputVoltage(motorVoltage);

    // update the models
    this.systemSim.update(Constants.loopPeriodSecs);

    this.motorSimState.setRawRotorPosition(
        PantherUtil.metersToRotations(systemSim.getPositionMeters()));

    this.motorSimState.setRotorVelocity(
        PantherUtil.metersToRotations(systemSim.getVelocityMetersPerSecond()));

    m_Mech.updateElevator(systemSim.getPositionMeters());

    Logger.recordOutput(
        "FinalComponentPoses1",
        new Pose3d[] {
          new Pose3d(0.07, 0.01, 0.146 + systemSim.getPositionMeters(), new Rotation3d(0, 0, 0))
        });

    Logger.recordOutput(
        "FinalComponentPoses2",
        new Pose3d[] {
          new Pose3d(
              0.1, 0.006, 0.178 + (systemSim.getPositionMeters() * 2), new Rotation3d(0, 0, 0))
        });

    Logger.recordOutput(
        "FinalComponentPoses3",
        new Pose3d[] {
          new Pose3d(
              0.32, 0.01, 0.501 + (systemSim.getPositionMeters() * 2), new Rotation3d(0, 0, 0))
        });
  }
}
