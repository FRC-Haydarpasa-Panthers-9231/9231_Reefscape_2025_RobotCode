package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
  public static final int kElevatorMotorLeadID = 9;
  public static final int kElevatorMotorFollowerID = 10;
  public static final Distance kDrivetrainHeight = Meters.of(Units.inchesToMeters(10));

  public static final double kElevatorGearing = 25;
  public static final double kCarriageMass = 18;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(0.62);
  public static final Distance kMinElevatorHeightMeters = Meters.of(0);
  public static final Distance kMaxElevatorHeightMeters = Meters.of(1.55); // 1.6075
  public static final double kDefaultSetpoint = 0;
  public static final String kSubsystemName = "Elevator";
  public static final boolean kIsInverted = false;
  public static final int kElevatorTeeth = 17;
  public static final double kElevatorPitch = 0.00635; // 6.35 mm = 0.00635 metre

  public static final double KP_SLOT0 = 0;
  public static final double KI_SLOT0 = 0;
  public static final double KD_SLOT0 = 0;
  public static final double KS_SLOT0 = 0;
  public static final double KV_SLOT0 = 0;
  public static final double KA_SLOT0 = 0;
  public static final double KG_SLOT0 = 0;

  public static final double KP_SLOT1 = 1000;
  public static final double KI_SLOT1 = 5;
  public static final double KD_SLOT1 = 50;
  public static final double KS_SLOT1 = 0;
  public static final double KV_SLOT1 = 0;
  public static final double KA_SLOT1 = 0;
  public static final double KG_SLOT1 = 1;

  public enum ReefBranch {
    L1,
    L2,
    L3,
    L4,

    ALGAE_1,
    ALGAE_2,
  }
}
