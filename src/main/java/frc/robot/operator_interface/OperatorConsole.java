// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class OperatorConsole implements OperatorInterface {
  private final CommandXboxController controller;

  public OperatorConsole(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public Trigger getElevatorZeroPositionButton() {
    return controller.a();
  }

  @Override
  public Trigger getElevatorL2PositionButton() {
    return controller.x();
  }

  @Override
  public Trigger getElevatorL3PositionButton() {
    return controller.b();
  }

  @Override
  public Trigger getElevatorL4PositionButton() {
    return controller.y();
  }

  @Override
  public Trigger getElevatorA1PositionButton() {
    return controller.pov(180);
  }

  @Override
  public Trigger getElevatorA2PositionButton() {
    return controller.pov(0);
  }

  @Override
  public Trigger getIntakingCoralButton() {
    return controller.leftBumper();
  }

  @Override
  public Trigger getScoringCoralButton() {
    return controller.rightBumper();
  }
}
