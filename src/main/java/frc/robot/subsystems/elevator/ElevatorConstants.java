package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;

public class ElevatorConstants {
  public static final int kElevatorMotorLeadID = 9;
  public static final int kElevatorMotorFollowerID = 10;
  public static final Distance kDrivetrainHeight = Meters.of(Units.inchesToMeters(10));
  public static final double TOLERANCE_METERS = 0;
  public static final Distance PULLY_CIRCUMFERENCE = Meters.of(Units.inchesToMeters(5.9055));

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
  public static final double kPullyCircumFerenceMeters = 2 * Math.PI * kElevatorDrumRadius;

  public static final double KP_SLOT0 = 0;
  public static final double KI_SLOT0 = 0;
  public static final double KD_SLOT0 = 0;
  public static final double KS_SLOT0 = 0;
  public static final double KV_SLOT0 = 0;
  public static final double KA_SLOT0 = 0;
  public static final double KG_SLOT0 = 0;

  public static final double KP_SLOT1 = 50;
  public static final double KI_SLOT1 = 5;
  public static final double KD_SLOT1 = 1;
  public static final double KS_SLOT1 = 0;
  public static final double KV_SLOT1 = 0;
  public static final double KA_SLOT1 = 0;
  public static final double KG_SLOT1 = 0.1;

  public static final Per<AngleUnit, DistanceUnit> MOTOR_ROTATIONS_PER_METER_UNIT =
      Rotations.of(1)
          .div(Inches.of(ElevatorConstants.kElevatorTeeth * ElevatorConstants.kElevatorPitch * 2));

  public static final double MOTOR_ROTATIONS_PER_METER =
      MOTOR_ROTATIONS_PER_METER_UNIT.in(Rotations.per(Meter));

  public static final Distance MIN_LENGTH = Inches.of(27.0);
  public static final Distance MAX_LENGTH = Inches.of(67.0);

  public static final double MIN_LENGTH_ROTATIONS =
      MIN_LENGTH.in(Meters) * MOTOR_ROTATIONS_PER_METER;
  public static final double MAX_LENGTH_ROTATIONS =
      MAX_LENGTH.in(Meters) * MOTOR_ROTATIONS_PER_METER;

  public enum ReefBranch {
    L1,
    L2,
    L3,
    L4,

    ALGAE_1,
    ALGAE_2,
  }

  /*
   * public static final double kForwardSoftLimitThreshold =
   * kMaxElevatorHeightMeters.in(Meters)
   * / 2
   * (ElevatorConstants.kElevatorTeeth * ElevatorConstants.kElevatorPitch);
   *
   * public static final double kReverseSoftLimitThreshold = 0;
   */

  public static final Distance L1_HEIGHT = Meters.of(Units.inchesToMeters(15.94));
  public static final Distance L2_HEIGHT = Meters.of(Units.inchesToMeters(31.39));
  public static final Distance L3_HEIGHT = Meters.of(Units.inchesToMeters(47.64));
  public static final Distance L4_HEIGHT = Meters.of(Units.inchesToMeters(72.05));
}
