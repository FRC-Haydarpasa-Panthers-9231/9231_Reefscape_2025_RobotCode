package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.subsystems.ElevatorRoller.ElevatorRollerConstants;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.SCORING_HEIGHT;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.subsystems.led.SUB_LED.LEDState;

public class ScoringCoral extends SequentialCommandGroup {

    Drive drive;
    SUB_ElevatoRoller elevatorRoller;
    SUB_Elevator elevator;
    SUB_LED leds;
    SCORING_HEIGHT scoringHeight;
    double coralOuttakeSpeed;
    ReefSide side;
    CommandXboxController driverController;

    public ScoringCoral(SUB_Elevator elevator, SUB_LED leds, SUB_ElevatoRoller elevatorRoller,
        Drive drive, CommandXboxController driverController,
        SCORING_HEIGHT scoringHeight, ReefSide side)
    {
        this.elevatorRoller = elevatorRoller;
        this.elevator = elevator;
        this.leds = leds;
        this.drive = drive;
        this.driverController = driverController;
        this.scoringHeight = scoringHeight;
        this.side = side;

        addCommands(
            new ParallelCommandGroup(
                Commands.run(
                    () -> DriveCommands.joystickApproach(drive, () -> -driverController.getLeftY(),
                        () -> FieldConstants.getNearestReefBranch(drive.getPose(), side))),
                new SequentialCommandGroup(
                    Commands.runOnce(() -> leds.setState(LEDState.SCORING_CORAL)),

                    // Asansör doğru konuma geldiğinde atış yap
                    Commands.waitUntil(() -> elevator.isAtSetPoint()),
                    Commands.runOnce(() -> elevatorRoller.setSpeed(getCoralOuttakeSpeed())),
                    Commands
                        .waitSeconds(ElevatorRollerConstants.kCoralScoreTime.in(Units.Seconds))))


        );
    }

    public double getCoralOuttakeSpeed()
    {
        if (scoringHeight.equals(SCORING_HEIGHT.CORAL_L1_HEIGHT)) {
            coralOuttakeSpeed = ElevatorRollerConstants.kL1ScoringSpeed;
        } else if (scoringHeight.equals(SCORING_HEIGHT.CORAL_L2_HEIGHT)) {
            coralOuttakeSpeed = ElevatorRollerConstants.kL2ScoringSpeed;
        } else if (scoringHeight.equals(SCORING_HEIGHT.CORAL_L3_HEIGHT)) {
            coralOuttakeSpeed = ElevatorRollerConstants.kL3ScoringSpeed;
        } else {
            coralOuttakeSpeed = ElevatorRollerConstants.kL4ScoringSpeed;
        }

        return coralOuttakeSpeed;
    }
}
