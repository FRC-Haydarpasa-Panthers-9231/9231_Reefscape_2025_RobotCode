package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private final Drive swerve;
    private SUB_Elevator elevator;
    SUB_Elevator elevatorRoller;
    private final SUB_ProcessorPivot processorPivot;
    private final SUB_ProcessorRoller processorRoller;
    private RobotContainer container;

    public enum WantedSuperState {
        ZERO_STATE,
        IDLE,
        CORAL_STAGE_1,
        CORAL_STAGE_2,
        CORAL_STAGE_3,
        CORAL_STAGE_4,
        ALGEA_STAGE_1,
        ALGEA_STAGE_2,
        ALGEA_GROUND,
        ALGEA_PROCESSOR,
    }

    public enum CurrentSuperState {
        ZERO_STATE,
        IDLE,
        CORAL_STAGE_1,
        CORAL_STAGE_2,
        CORAL_STAGE_3,
        CORAL_STAGE_4,
        ALGEA_STAGE_1,
        ALGEA_STAGE_2,
        ALGEA_GROUND,
        ALGEA_PROCESSOR,
    }

    private WantedSuperState wantedSuperState = WantedSuperState.ZERO_STATE;
    private CurrentSuperState currentSuperState = CurrentSuperState.ZERO_STATE;
    private CurrentSuperState previousSuperState;

    public Superstructure(
        Drive swerve,
        SUB_Elevator elevator,
        SUB_ElevatoRoller elevatoRoller,
        SUB_ProcessorPivot processorPivot,
        SUB_ProcessorRoller processorRoller,
        RobotContainer container)
    {
        this.swerve = swerve;
        this.elevator = elevator;
        this.elevatorRoller = elevatorRoller;
        this.processorPivot = processorPivot;
        this.processorRoller = processorRoller;
        this.container = container;
    }

    @Override
    public void periodic()
    {

        Logger.recordOutput("DesiredSuperstate", wantedSuperState);
        currentSuperState = handleStateTransitions();
        applyStates();

        if (currentSuperState != previousSuperState) {
            Logger.recordOutput("CurrentSuperstate", currentSuperState);
        }
    }

    private CurrentSuperState handleStateTransitions()
    {

        previousSuperState = currentSuperState;

        switch (wantedSuperState) {
            case ZERO_STATE:
                currentSuperState = CurrentSuperState.ZERO_STATE;
                break;

            case IDLE:
                currentSuperState = CurrentSuperState.IDLE;
                break;

            case CORAL_STAGE_1:
                currentSuperState = CurrentSuperState.CORAL_STAGE_1;
                break;

            case CORAL_STAGE_2:
                currentSuperState = CurrentSuperState.CORAL_STAGE_2;
                break;

            case CORAL_STAGE_3:
                currentSuperState = CurrentSuperState.CORAL_STAGE_3;
                break;

            case CORAL_STAGE_4:
                currentSuperState = CurrentSuperState.CORAL_STAGE_4;
                break;

            case ALGEA_STAGE_1:
                currentSuperState = CurrentSuperState.ALGEA_STAGE_1;
                break;

            case ALGEA_STAGE_2:
                currentSuperState = CurrentSuperState.ALGEA_STAGE_2;
                break;

            case ALGEA_GROUND:
                currentSuperState = CurrentSuperState.ALGEA_GROUND;
                break;

            case ALGEA_PROCESSOR:
                currentSuperState = CurrentSuperState.ALGEA_PROCESSOR;
                break;

            default:
                currentSuperState = CurrentSuperState.ZERO_STATE;
        }
        return currentSuperState;
    }

    private void applyStates()
    {
        switch (currentSuperState) {
            case ZERO_STATE:
                handleZeroStates();
                break;
            case IDLE:
                handleIdling();

            case CORAL_STAGE_1:
                handleCoralStage1();

            case CORAL_STAGE_2:
                handleCoralStage2();
                break;

            case CORAL_STAGE_3:
                handleCoralStage3();
                break;

            case CORAL_STAGE_4:
                handleCoralStage4();
                break;

            case ALGEA_STAGE_1:
                handleAlgeaStage1();
                break;

            case ALGEA_STAGE_2:
                handleAlgeaStage2();
                break;

            case ALGEA_GROUND:
                handleAlgeaGround();
                break;

            case ALGEA_PROCESSOR:
                handleAlgeaProcessor();
                break;

        }
    }

    private void handleZeroStates()
    {
        elevator.setWantedState(SUB_Elevator.WantedState.ZERO_ELEVATOR);
        processorRoller.setWantedState(SUB_ProcessorRoller.WantedState.OFF);
    }

    private void handleIdling()
    {
        elevator.setWantedState(SUB_Elevator.WantedState.ZERO_ELEVATOR);
        processorRoller.setWantedState(SUB_ProcessorRoller.WantedState.OFF);
    }

    private void handleCoralStage1()
    {


    }

    private void handleCoralStage2()
    {


    }

    private void handleCoralStage3()
    {


    }

    private void handleCoralStage4()
    {


    }

    private void handleAlgeaStage1()
    {


    }

    private void handleAlgeaStage2()
    {


    }

    private void handleAlgeaGround()
    {


    }

    private void handleAlgeaProcessor()
    {


    }



    /** State pushers */
    public void setWantedSuperState(WantedSuperState wantedSuperState)
    {
        this.wantedSuperState = wantedSuperState;
    }

    public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState)
    {
        return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
    }
}
