package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverInterface {

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getDriveToNearestCoralStationButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDriveToNearestReefFaceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDriveToNearestReefLeftBranchButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDriveToNearestReefRightBranchButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCurrentPoseButton() {
    return new Trigger(() -> false);
  }
}
