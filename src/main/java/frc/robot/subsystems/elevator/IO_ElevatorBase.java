package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ElevatorBase {

    @AutoLog
    public static class ElevatorInputs {
        public double[] elevatorAppliedVolts = new double[] {};
        public double[] elevatorCurrentAmps = new double[] {};
        public double[] elevatorTempCelcius = new double[] {};
        public double elevatorPositionRad = 0.0;
        public double elevatorVelocityRadPerSec = 0.0;
    }

    void updateInputs(ElevatorInputs inputs);

    /**
     * Asansör motorlarının voltajını ayarlar
     * 
     * @param volts Çalıştırmak istediğiniz volt değeri.
     */
    public void setElevatorVoltage(double volts);

    /**
     * PID kontrolünde kullanılacak sabitler
     * 
     * @param kG
     * @param kS
     * @param kV
     * @param kA
     * @param kP
     * @param kI
     * @param kD
     */
    public void setSlot0(double kG, double kS, double kV, double kA, double kP, double kI,
        double kD);

    /**
     * Asansör motorlarını durdurur.
     */
    public void stopMotor();

    /**
     * Asansörü belli bir pozisyona götürür.
     * 
     * @param positionRad başlangıç konumundan motorun radyan cinsinden sabitlenmek istendiği konum.
     */
    public void runPosition(double positionRad);

    /**
     * Asansöre yazılımsal limit ekler.
     * 
     * @param positionRad Asansörün gitmek istediği konum
     */
    public boolean isPositionWithinLimits(double positionRad);
}
