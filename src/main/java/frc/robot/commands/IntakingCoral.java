package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorRoller.ElevatorRollerConstants;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;

public class IntakingCoral extends Command {
    private final SUB_ElevatoRoller elevatoRoller;

    // private boolean lastSensorState = false; // Önceki sensör durumu
    // private int triggerCount = 0; // Kaç kez değiştiğini takip eder

    public IntakingCoral(SUB_ElevatoRoller elevatoRoller)
    {
        this.elevatoRoller = elevatoRoller;
        addRequirements(elevatoRoller);
    }

    @Override
    public void initialize()
    {
        // lastSensorState = elevatoRoller.hasCoral(); // Başlangıçtaki sensör durumunu al
        // triggerCount = 0; // Tetikleme sayacını sıfırla
        elevatoRoller.setSpeed(ElevatorRollerConstants.kIntakingSpeed);
    }

    @Override
    public void end(boolean interrupted)
    {
        new SequentialCommandGroup(
            Commands.runOnce(
                () -> elevatoRoller.setSpeed(ElevatorRollerConstants.kAfterSensorIntakingSpeed),
                elevatoRoller),
            new WaitCommand(ElevatorRollerConstants.kAfterSensorWaitTime),
            Commands.runOnce(() -> elevatoRoller.setSpeed(0), elevatoRoller));
    }

    @Override
    public boolean isFinished()
    {
        return elevatoRoller.hasCoral();
    }

    /*
     * @Override
     * public void execute() {
     * boolean currentSensorState = elevatoRoller.hasCoral();
     *
     * // Sensör değiştiğinde (false -> true veya true -> false)
     * if (currentSensorState != lastSensorState) {
     * triggerCount++; // Tetiklenme sayısını artır
     *
     * if (triggerCount == 1) {
     * elevatoRoller.setSpeed(0.1); // İlk tetiklemede yavaşlat
     * } else if (triggerCount == 2) {
     * elevatoRoller.setSpeed(0.0); // İkinci tetiklemede tamamen durdur
     * }
     * }
     *
     * lastSensorState = currentSensorState; // Sensör durumunu güncelle
     * }
     *
     * @Override
     * public boolean isFinished() {
     * return triggerCount >= 2; // 2 kez tetiklendiyse komut tamamlanır
     * }
     *
     * @Override
     * public void end(boolean interrupted) {
     * elevatoRoller.setSpeed(0.0); // Komut bittiğinde motoru durdur
     * }
     */
}
