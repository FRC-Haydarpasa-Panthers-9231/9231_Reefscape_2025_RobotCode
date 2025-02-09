package frc.robot.util;

public class PantherUtil {

  /**
   * Rotasyon cinsinden pozisyonu metreye çevirir.
   *
   * @param positionRotations Rotasyon cinsinden pozisyon.
   * @param drumRadius Tambur yarıçapı (metre cinsinden).
   * @param gearRatio Dişli oranı.
   * @return Metre cinsinden yükseklik.
   */
  public static double rotationsToMeters(
      double positionRotations, double drumRadius, double gearRatio) {
    return positionRotations * (2 * Math.PI * drumRadius) / gearRatio;
  }

  /**
   * Metre cinsinden yüksekliği rotasyona çevirir.
   *
   * @param positionMeters Metre cinsinden yükseklik.
   * @param drumRadius Tambur yarıçapı (metre cinsinden).
   * @param gearRatio Dişli oranı.
   * @return Rotasyon cinsinden pozisyon.
   */
  public static double metersToRotations(
      double positionMeters, double drumRadius, double gearRatio) {
    return positionMeters / (2 * Math.PI * drumRadius) * gearRatio;
  }
}
