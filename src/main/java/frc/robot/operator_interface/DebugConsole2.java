package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DebugConsole2 implements DebugInterface2 {
  private final CommandXboxController controller;

  public DebugConsole2(int port) {
    controller = new CommandXboxController(port);
  }
}
