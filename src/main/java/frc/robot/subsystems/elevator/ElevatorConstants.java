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

    // TODO: IDLERİ BELİRLE VE VER

    public static final int kElevatorMotorLeadID = 9;
    public static final int kElevatorMotorFollowerID = 10;

    // TODO: BU DEĞERİ AL
    public static final Distance kDrivetrainHeight =
        Meters.of(edu.wpi.first.math.util.Units.inchesToMeters(10));

    // TODO GEREKİRSE TOLERANSI ARTTIR
    public static final Distance kTolerance = Meters.of(0.003);


    // TODO ELEVATOR YAPILDIKTAN SONRA TAM REDUCTION'I AL
    public static final double kElevatorGearing = 25;
    public static final double kCarriageMass = 18;
    public static final double kElevatorDrumRadius =
        edu.wpi.first.math.util.Units.inchesToMeters(0.62);
    public static final Distance kMinElevatorHeightMeters = Meters.of(0);
    public static final Distance kMaxElevatorHeightMeters = Meters.of(1.55); // 1.6075
    public static final double kDefaultSetpoint = 0;
    public static final String kSubsystemName = "Elevator";

    // TODO INVERTLENIP INVERTLENMEYECEĞİNİ KONTROL ET
    public static final boolean kIsInverted = false;

    public static final int kElevatorTeeth = 17;
    public static final double kElevatorPitch = 0.00635; // 6.35 mm = 0.00635 metre


    // TODO GERÇEK HAYAT PID VE FEEDFORWARDLARINI AYARLA.
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

    public static final double kForwardLimit = Units.Meters.of(1.4).in(Units.Meters);
    public static final double kReverseLimit = Units.Meters.of(0).in(Units.Meters);


    public static TalonFXConfiguration kElavatorConfig = new TalonFXConfiguration();

    static {
        kElavatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kElavatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // TODO: SOFTWARE LİMİTLERİNİ BUL VE EKLE
        kElavatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kElavatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            Units.Meters.of(1.4).in(Units.Meters);
        kElavatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kElavatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            Units.Meters.of(0).in(Units.Meters);

        kElavatorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        kElavatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kElavatorConfig.CurrentLimits
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(50));

        kElavatorConfig.Voltage.PeakForwardVoltage = 12.0;
        kElavatorConfig.Voltage.PeakReverseVoltage = -12;

        kElavatorConfig.Feedback.SensorToMechanismRatio = kElevatorGearing;

        kElavatorConfig.Slot0.kG = KP_SLOT0; // Volts to overcome gravity
        kElavatorConfig.Slot0.kS = KS_SLOT0; // Volts to overcome static friction
        kElavatorConfig.Slot0.kV = KV_SLOT0; // Volts for a velocity target of 1 rps
        kElavatorConfig.Slot0.kA = KA_SLOT0; // Volts for an acceleration of 1 rps/s
        kElavatorConfig.Slot0.kP = KP_SLOT0;
        kElavatorConfig.Slot0.kI = KI_SLOT0;
        kElavatorConfig.Slot0.kD = KD_SLOT0;

        kElavatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        kElavatorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        kElavatorConfig.Slot1.kG = 0.1; // Volts to overcome gravity
        kElavatorConfig.Slot1.kS = 0.4; // Volts to overcome static friction
        kElavatorConfig.Slot1.kV = 0.001; // Volts for a velocity target of 1 rps
        kElavatorConfig.Slot1.kA = 0.0; // Volts for an acceleration of 1 rps/s
        kElavatorConfig.Slot1.kP = 300;
        kElavatorConfig.Slot1.kI = 1;
        kElavatorConfig.Slot1.kD = 50;

        kElavatorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        kElavatorConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        // TODO MOTİON MAGİC DEĞERLERİNİ DÜZENLE
        kElavatorConfig.MotionMagic.MotionMagicCruiseVelocity = 200;
        kElavatorConfig.MotionMagic.MotionMagicAcceleration = 400;
        kElavatorConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    }

    public static TalonFXConfiguration kCoastModeConfiguration = new TalonFXConfiguration();

    static {
        kCoastModeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kCoastModeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }


    // TODO ZEROİNG VOLTAGE YETERLİ DEĞİLSE DÜZENLE
    public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(-1);


    // TODO: SCORİNG HEİGHTLERI ÖLÇ
    public enum SCORING_HEIGHT {
        CORAL_L1_HEIGHT(19.0),
        CORAL_L2_HEIGHT(19.0),
        CORAL_L3_HEIGHT(34.75),
        CORAL_L4_HEIGHT(61.0),
        ALGAE_PREP_PROCESSOR_HEIGHT(1.0),
        ALGAE_L3_CLEANING(25.0),
        ALGAE_L2_CLEANING(9.0),
        PREP_0(0.0),
        DEADZONE_DISTANCE(1.0),
        CORAL_INTAKE_HEIGHT(0.0);

        private final double heightInMeters;

        // Constructor for the enum
        SCORING_HEIGHT(double heightInMeters)
        {
            this.heightInMeters = heightInMeters;
        }

        // Getter to retrieve the height value in meters
        public double getHeightInMeters()
        {
            return heightInMeters;
        }
    }

    public static final Time kZeroingTimeout = Units.Seconds.of(3);

    public static final AngularVelocity kManualZeroingStartVelocity =
        Units.RotationsPerSecond.of(5);
    public static final AngularVelocity kManualZeroingDeltaVelocity =
        Units.RotationsPerSecond.of(5);

    /** The voltage supplied to the motor in order to zero */
    public static final Voltage kZeroingVoltage = Units.Volts.of(-1);

    /**
     * The value that the motor reports when it is at it's zeroed position. This may not necessarily
     * be 0 due to mechanical slop
     */
    public static final Distance ZEROED_POS = Units.Meters.of(0);


    // TODO: GEREKİSE BU DEĞERİ DÜZENLE
    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer continue in that
     * direction)
     */
    public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

    /** The elapsed time required to consider the motor as zeroed */
    public static final Time ZEROED_TIME = Units.Seconds.of(1);
}
