package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {

    public static final int kElevatorMotorLeadID = 9;
    public static final int kElevatorMotorFollowerID = 10;

    // TODO: BU DEgERİ AL
    public static final Distance kDrivetrainHeight =
        Meters.of(edu.wpi.first.math.util.Units.inchesToMeters(10));

    // TODO GEREKİRSE TOLERANSI ARTTIR
    public static final double kTolerance = 0.030;

    // TODO ELEVATOR YAPILDIKTAN SONRAs TAM REDUCTION'I AL
    public static final double kElevatorGearing = 9;
    public static final double kCarriageMass = 18;
    public static final double kElevatorDrumRadius =
        edu.wpi.first.math.util.Units.inchesToMeters(0.62);
    public static final Distance kMinElevatorHeightMeters = Meters.of(0);
    public static final Distance kMaxElevatorHeightMeters = Meters.of(100); // 1.6075
    public static final double kDefaultSetpoint = 0;
    public static final String kSubsystemName = "Elevator";

    public static final int kElevatorTeeth = 17;
    public static final double kElevatorPitch = 0.00635; // 6.35 mm = 0.00635 metre

    // TODO GERÇEK HAYAT PID VE FEEDFORWARDLARINI AYARLA.
    public static final double KP_SLOT0 = 24.976;
    public static final double KI_SLOT0 = 0;
    public static final double KD_SLOT0 = 1.9351;
    public static final double KS_SLOT0 = 0.092144;
    public static final double KV_SLOT0 = 1.1236;
    public static final double KA_SLOT0 = 0.045095;
    public static final double KG_SLOT0 = 0.50164;

    public static final double KP_SLOT1 = 77;
    public static final double KI_SLOT1 = 0;
    public static final double KD_SLOT1 = 3.91;
    public static final double KS_SLOT1 = 0;
    public static final double KV_SLOT1 = 2.55;
    public static final double KA_SLOT1 = 0.200;
    public static final double KG_SLOT1 = 0.359;
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 210;
    public static final double MOTION_MAGIC_ACCELERATION = 60;
    public static final double MOTION_MAGIC_JERK = 18;

    public enum ReefBranch {
        L1,
        L2,
        L3,
        L4,

        ALGAE_1,
        ALGAE_2,
    }

    public static final double kForwardLimit = 5.97;
    public static final double kReverseLimit = 0;

    public static TalonFXConfiguration kElavatorConfig = new TalonFXConfiguration();

    static {
        kElavatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kElavatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // TODO: SOFTWARE LIMITLERINI BUL VE EKLE
        kElavatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kElavatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardLimit;
        kElavatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kElavatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseLimit;
        kElavatorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        kElavatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kElavatorConfig.CurrentLimits
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(57));

        kElavatorConfig.Voltage.PeakForwardVoltage = 12.0;
        kElavatorConfig.Voltage.PeakReverseVoltage = -12;

        kElavatorConfig.Feedback.SensorToMechanismRatio = kElevatorGearing;

        kElavatorConfig.Slot0.kG = KG_SLOT0; // Volts to overcome gravity
        kElavatorConfig.Slot0.kS = KS_SLOT0; // Volts to overcome static friction
        kElavatorConfig.Slot0.kV = KV_SLOT0; // Volts for a velocity target of 1 rps
        kElavatorConfig.Slot0.kA = KA_SLOT0; // Volts for an acceleration of 1 rps/s
        kElavatorConfig.Slot0.kP = KP_SLOT0;
        kElavatorConfig.Slot0.kI = KI_SLOT0;
        kElavatorConfig.Slot0.kD = KD_SLOT0;

        kElavatorConfig.MotionMagic.MotionMagicExpo_kA = 0.35;
        kElavatorConfig.MotionMagic.MotionMagicExpo_kV = 0.35;
        kElavatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        kElavatorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        kElavatorConfig.Slot1.kG = KG_SLOT1; // Volts to overcome gravity
        kElavatorConfig.Slot1.kS = KS_SLOT1; // Volts to overcome static friction
        kElavatorConfig.Slot1.kV = KV_SLOT1; // Volts for a velocity target of 1 rps
        kElavatorConfig.Slot1.kA = KA_SLOT1; // Volts for an acceleration of 1 rps/s
        kElavatorConfig.Slot1.kP = KP_SLOT1;
        kElavatorConfig.Slot1.kI = KI_SLOT1;
        kElavatorConfig.Slot1.kD = KD_SLOT1;

        kElavatorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        kElavatorConfig.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        kElavatorConfig.Slot2.kG = 0.4; // Volts to overcome gravity
        kElavatorConfig.Slot2.kS = 0; // Volts to overcome static friction
        kElavatorConfig.Slot2.kV = 2.40; // Volts for a velocity target of 1 rps
        kElavatorConfig.Slot2.kA = 0.8; // Volts for an acceleration of 1 rps/s
        kElavatorConfig.Slot2.kP = 150;
        kElavatorConfig.Slot2.kI = 0;
        kElavatorConfig.Slot2.kD = 4;

        kElavatorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;

        kElavatorConfig.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        // kElavatorConfig.MotionMagic.MotionMagicExpo_kV = MOTION_MAGIC_EXPO_KV;
    }

    public static TalonFXConfiguration kCoastModeConfiguration = new TalonFXConfiguration();

    static {
        kCoastModeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kCoastModeConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    // TODO ZEROİNG VOLTAGE YETERLİ DEgİLSE DÜZENLE
    public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(-1);

    // TODO: SCORİNG HEİGHTLERI ÖLÇ
    public enum ELEVATOR_HEIGHT {
        ZERO_HEIGHT(0.0),
        ALGAE_GROUND_INTAKE(0.0),
        CORAL_L1_HEIGHT(0),
        CORAL_L2_HEIGHT(7.7),
        CORAL_L3_HEIGHT(19.5),
        CORAL_L4_HEIGHT(36.5),
        ALGAE_PREP_PROCESSOR_HEIGHT(1.0),
        ALGAE_L3_CLEANING(25.0),
        ALGAE_L2_CLEANING(9.0),
        DEADZONE_DISTANCE(1.0),
        CORAL_INTAKE_HEIGHT(0.0);

        private final double positionRads;

        // Constructor for the enum
        ELEVATOR_HEIGHT(double positionRads)
        {
            this.positionRads = positionRads;
        }

        // Getter to retrieve the height value in meters
        public double getPositionRads()
        {
            return positionRads;
        }
    }

    public static final Time kZeroingTimeout = Units.Seconds.of(3);

    /**
     * The value that the motor reports when it is at it's zeroed position. This may not necessarily
     * be 0 due to mechanical slop
     */
    public static final double ZEROED_POS = 0;

    // TODO: GEREKİSE BU DEgERİ DÜZENLE
    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer continue in that
     * direction)
     */
    public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

    /** The elapsed time required to consider the motor as zeroed */
    public static final Time ZEROED_TIME = Units.Seconds.of(1);
}
