// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ResetOdometryCommand extends Command {

  Drive drive;
  Vision vision;

  /** Creates a new ResetOdometryCommand. */
  public ResetOdometryCommand(Drive drive, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d visionPose = vision.getRobotPose();
    drive.setPose(visionPose);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
