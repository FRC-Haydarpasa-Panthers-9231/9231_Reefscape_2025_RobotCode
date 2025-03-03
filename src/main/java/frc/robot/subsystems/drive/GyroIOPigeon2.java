// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          TunerConstants.DrivetrainConstants.Pigeon2Id,
          TunerConstants.DrivetrainConstants.CANBusName);
  private final StatusSignal<Angle> yaw = pigeon.getYaw().clone();
  private final StatusSignal<Angle> pitch = pigeon.getPitch().clone();
  private final StatusSignal<Angle> roll = pigeon.getRoll().clone();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());

    FaultReporter.getInstance().registerHardware("Drivetrain", "Pigeon", pigeon);
  }

  public void addToSmartDashboard() {
    SmartDashboard.putData(
        "Gyro",
        builder -> {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", this::getCompassHeading, null);
        });
  }

  public double getHeading() {
    return pigeon.getYaw().getValueAsDouble();
  }

  /**
   * The heading value from the pigeon increases counterclockwise (0 North, 90 West, 180 South, 270
   * East) Some features need degrees to look like a compass, increasing clockwise (0 North, 90
   * East, 180 South, 270 West) with rollover (max value 359.99, min 0)
   *
   * <p>Returns heading from pigeon from 0 to 359.99 turning clockwise
   */
  public double getCompassHeading() {
    double heading = getHeading();

    // Get the heading. If the value is negative, need to flip it positive
    // also need to subtract from 360 to flip value to the correct compass heading
    heading = heading < 0 ? (0 - heading % 360) : (360 - (heading % 360));

    // 0 degree will result in 360, so set it back to 0
    heading = heading == 360.0 ? 0 : heading;

    return heading;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.yaw = yaw.getValueAsDouble();
    inputs.pitch = pitch.getValueAsDouble();
    inputs.roll = roll.getValueAsDouble();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
