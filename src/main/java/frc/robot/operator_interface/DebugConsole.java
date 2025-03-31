package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DebugConsole implements DebugInterface {
  private final CommandXboxController controller;

  public DebugConsole(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getTranslateX() {
    return -controller.getLeftX();
  }

  @Override
  public double getTranslateY() {
    return -controller.getLeftY();
  }

  @Override
  public double getRotate() {
    return controller.getRightX();
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return controller.a();
  }

  @Override
  public Trigger getResetGyroButton() {
    return controller.b();
  }

  @Override
  public Trigger getXStanceButton() {
    return controller.x();
  }

  @Override
  public Trigger getTranslationSlowModeButton() {
    return controller.y();
  }

  @Override
  public Trigger getRotationSlowModeButton() {
    return controller.y();
  }

  @Override
  public Trigger getLock180Button() {
    return controller.start();
  }

  @Override
  public Trigger getDriveToNearestCoralStationButton() {
    return new Trigger(controller.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.5));
  }

  @Override
  public Trigger getDriveToNearestReefFaceButton() {
    return new Trigger(controller.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.5));
  }

  @Override
  public Trigger getDriveToNearestReefLeftBranchButton() {
    return controller.leftBumper();
  }

  @Override
  public Trigger getDriveToNearestReefRightBranchButton() {
    return controller.rightBumper();
  }
}
