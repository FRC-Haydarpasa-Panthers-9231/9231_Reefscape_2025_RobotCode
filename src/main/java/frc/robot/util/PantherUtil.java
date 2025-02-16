package frc.robot.util;

import frc.robot.subsystems.elevator.ElevatorConstants;

public class PantherUtil {

  /** Converts mechanism rotations to meters. */
  public static double rotationsToMeters(double positionRotations) {
    double distancePerRotation =
        (2 * ElevatorConstants.kElevatorTeeth * ElevatorConstants.kElevatorPitch)
            / ElevatorConstants.kElevatorGearing; // Gear ratio uygulanıyor

    return positionRotations * distancePerRotation;
  }

  /** Converts meters to mechanism rotations. */
  public static double metersToRotations(double meters) {
    double distancePerRotation =
        (2 * ElevatorConstants.kElevatorTeeth * ElevatorConstants.kElevatorPitch)
            / ElevatorConstants.kElevatorGearing; // Gear ratio uygulanıyor

    return meters / distancePerRotation;
  }
}
