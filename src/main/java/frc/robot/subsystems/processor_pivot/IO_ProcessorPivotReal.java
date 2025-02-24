package frc.robot.subsystems.processor_pivot;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkUtil;

public class IO_ProcessorPivotReal implements IO_ProcessorPivotBase {



    // FIXME: DOgRU PID DEgERLERİNİ BUL
    private static final LoggedTunableNumber kP =
        new LoggedTunableNumber("Processor_Pivot/kP", ProcessorPivotConstants.kP);
    private static final LoggedTunableNumber kI =
        new LoggedTunableNumber("Processor_Pivot/kI", ProcessorPivotConstants.kD);
    private static final LoggedTunableNumber kD =
        new LoggedTunableNumber("Processor_Pivot/kD", ProcessorPivotConstants.kI);

    SparkMax processorPivot;
    private PIDController processorPivotPID;

    static DutyCycleEncoder absoluteEncoder =
        new DutyCycleEncoder(ProcessorPivotConstants.kProcessorPivotEncoderChannel);

    SparkMaxConfig config = new SparkMaxConfig();

    private final Alert configAlert =
        new Alert("Processor pivot için config ayarlanırken bir hata oluştu.", AlertType.kError);

    public double degreeAim;

    public IO_ProcessorPivotReal()
    {
        processorPivot =
            new SparkMax(ProcessorPivotConstants.kProcessorPivotMotorID, MotorType.kBrushless);

        processorPivotPID = new PIDController(kP.get(), kI.get(), kD.get());



        SparkUtil.tryUntilOk(
            processorPivot,
            5,
            () -> processorPivot.configure(
                config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(ProcessorPivotConstants.kIsInverted),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
            configAlert);



        FaultReporter.getInstance()
            .registerHardware(
                ProcessorPivotConstants.kSubsystemName, "Processor Pivot Motor", processorPivot);
    }

    private void PIDinitialize(double degree)
    {
        degreeAim = degree;
        processorPivotPID.reset();
        processorPivotPID.setSetpoint(degree);
        processorPivotPID.setTolerance(0.001);
    }

    @Override
    public void setSpeed(double speed)
    {
        processorPivot.set(speed);
    }

    @Override
    public void stopMotor()
    {
        processorPivot.stopMotor();
    }

    @Override
    public double getProcessorPivotPosition()
    {
        double degree = (absoluteEncoder.get() < 0.5f)
            ? absoluteEncoder.get() + 1
            : absoluteEncoder.get();
        return degree;

    }

    @Override
    public void setPosition(double setPoint)
    {
        if (getProcessorPivotPosition() < ProcessorPivotConstants.kProcessorPivotMaxAngle
            && getProcessorPivotPosition() > ProcessorPivotConstants.kProcessorPivotMinAngle) {
            PIDinitialize(setPoint);
        } else {
            stopMotor();
        }
    }

    @Override
    public boolean isAtSetpoint()
    {

        return processorPivotPID.atSetpoint();
    }


    @Override
    public void updateInputs(ProcessorPivotInputs inputs)
    {

        LoggedTunableNumber.ifChanged(hashCode(), values -> {
            processorPivotPID.setP(values[0]);
            processorPivotPID.setI(values[1]);
            processorPivotPID.setD(values[2]);

        }, kP, kI, kD);

        inputs.processorPivotAppliedVolts =
            processorPivot.getAppliedOutput() * processorPivot.getBusVoltage();
        inputs.processorPivotCurrentAmps = processorPivot.getOutputCurrent();
        inputs.processorPivotPositionRads = Units.rotationsToRadians(getProcessorPivotPosition());
        inputs.processorPivotTempCelcius = processorPivot.getMotorTemperature();
    }
}
