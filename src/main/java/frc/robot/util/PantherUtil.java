package frc.robot.util;

import frc.robot.Constants;

public class PantherUtil {

  /**
   * Rotasyon cinsinden pozisyonu metreye çevirir.
   *
   * @param positionRotations Rotasyon cinsinden pozisyon.
   * @param teeth Zincir dişlisinin diş sayısı.
   * @param pitch Zincirin adımı (metre cinsinden).
   * @return Metre cinsinden yükseklik.
   */
  public static double rotationsToMeters(double positionRotations) {
    // Motorun bir turunda asansörün yükselme miktarı = 2 * (teeth * pitch)
    double distancePerRotation =
        2 * (Constants.Elevator.kElevatorTeeth * Constants.Elevator.kElevatorPitch);
    return positionRotations * distancePerRotation;
  }

  /**
   * Metre cinsinden yüksekliği rotasyona çevirir.
   *
   * @param positionMeters Metre cinsinden yükseklik.
   * @param teeth Zincir dişlisinin diş sayısı.
   * @param pitch Zincirin adımı (metre cinsinden).
   * @return Rotasyon cinsinden pozisyon.
   */
  public static double metersToRotations(double positionMeters) {
    // Motorun bir turunda asansörün yükselme miktarı = 2 * (teeth * pitch)
    double distancePerRotation =
        2 * (Constants.Elevator.kElevatorTeeth * Constants.Elevator.kElevatorPitch);
    return positionMeters / distancePerRotation;
  }
}
