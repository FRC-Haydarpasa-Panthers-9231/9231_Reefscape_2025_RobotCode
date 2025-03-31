package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public default Trigger getElevatorZeroPositionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getElevatorL2PositionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getElevatorL3PositionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getElevatorL4PositionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getElevatorA1PositionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getElevatorA2PositionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakingCoralButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getScoringCoralButton() {
    return new Trigger(() -> false);
  }
}
