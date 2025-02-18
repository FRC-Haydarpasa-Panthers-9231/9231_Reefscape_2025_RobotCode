package frc.robot.subsystems.processor_roller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

import frc.robot.util.SparkUtil;

public class IO_ProcessorRollerReal implements IO_ProcessorRollerBase {

    private SparkMax processorRoller;


    private boolean hasAlgaeOverride = false;


    public IO_ProcessorRollerReal()
    {
        processorRoller =
            new SparkMax(ProcessorRollerConstants.kProcessorRollerPort, MotorType.kBrushless);

        SparkUtil.tryUntilOk(
            processorRoller,
            5,
            () -> processorRoller.configure(
                new SparkMaxConfig()
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(false),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters),
            super.getClass().getName());
    }

    @Override
    public void updateInputs(ProcessorRollerInputs inputs)
    {
        inputs.processorRollerAppliedVolts =
            processorRoller.getAppliedOutput() * processorRoller.getBusVoltage();
        inputs.processorRollerCurrentAmps = processorRoller.getOutputCurrent();
        Logger.recordOutput("ProcessorRoller/hasAlgeaOverride", hasAlgaeOverride);
    }

    @Override
    public void setProcessorRollerSpeed(double speed)
    {
        processorRoller.set(speed);
    }

    @Override
    public void stopMotor()
    {
        processorRoller.stopMotor();
    }

    public boolean hasAlgae()
    {
        // Motorun mevcut çektiği akım değerini alıyoruz
        double intakeCurrent = processorRoller.getOutputCurrent();

        // Motorun uyguladığı çıkış voltajını alıyoruz
        double appliedVoltage =
            processorRoller.getAppliedOutput() * processorRoller.getBusVoltage();

        // Eşik değerler (constAlgaeIntake'ten alınan sabitler)
        double processorRollerHasGamePieceCurrent =
            ProcessorRollerConstants.kAlgeaIntakeHasGamePieceCurrent;

        double intakeHasGamePieceVoltage =
            ProcessorRollerConstants.kAlgeaProcessorRollerHasGamePieceVoltage;

        // Manuel override kontrolü
        if (hasAlgaeOverride) {
            return hasAlgaeOverride;
        }

        // Algae algılama koşulları:
        // 1. Akım belirli bir eşik değerden büyükse
        // 2. Motor voltajı belirli bir eşik değerden küçükse (yük altındaysa yavaşlama göstergesi)
        if ((intakeCurrent >= processorRollerHasGamePieceCurrent)
            && (appliedVoltage <= intakeHasGamePieceVoltage)) {
            return true;
        } else {
            return false;
        }

    }

    public void setHasAlgaeOverride(boolean passedHasGamePiece)
    {
        hasAlgaeOverride = passedHasGamePiece;
    }

    public void algaeToggle()
    {
        this.hasAlgaeOverride = !hasAlgaeOverride;
    }

    public double getAlgaeIntakeVoltage()
    {
        return processorRoller.getAppliedOutput() * processorRoller.getBusVoltage();
    }

    public void setAlgaeIntakeVoltage(double voltage)
    {
        processorRoller.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }
}
