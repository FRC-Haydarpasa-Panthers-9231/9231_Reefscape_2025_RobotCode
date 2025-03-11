// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Field2d;
import frc.robot.Field2d.Side;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class DriveToPose extends Command {
  private final Drive drivetrain;
  private Pose2d targetPose;
  private Transform2d targetTolerance;

  private boolean running = false;
  private Timer timer;
  private double timeout = 5;
  Side side;
  final ProfiledPIDController xController =
      new ProfiledPIDController(
          3.8, 0, 0.001, new TrapezoidProfile.Constraints(4, 3), Constants.loopPeriodSecs);

  final ProfiledPIDController yController =
      new ProfiledPIDController(
          3.8, 0, 0.001, new TrapezoidProfile.Constraints(4, 3), Constants.loopPeriodSecs);

  final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          3, 0, 0, new TrapezoidProfile.Constraints(4, 3), Constants.loopPeriodSecs);

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToPose(Drive drivetrain, Transform2d tolerance, Side side) {
    this.drivetrain = drivetrain;
    this.targetTolerance = tolerance;
    this.timer = new Timer();
    this.side = side;
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * s This method is invoked once when this command is scheduled. It resets all the PID controllers
   * and initializes the current and target poses. It is critical that this initialization occurs in
   * this method and not the constructor as this object is constructed well before the command is
   * scheduled and the robot's pose will definitely have changed and the target pose may not be
   * known until this command is scheduled.
   */
  @Override
  public void initialize() {
    // Reset all controllers
    Pose2d currentPose = drivetrain.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
    this.targetPose = frc.robot.Field2d.getInstance().getNearestBranch(side, drivetrain.getPose());

    Logger.recordOutput("DriveToPose/targetPose", targetPose);
    this.timer.restart();
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    // set running to true in this method to capture that the calculate method has been invoked
    // on
    // the PID controllers. This is important since these controllers will return true for
    // atGoal if
    // the calculate method has not yet been invoked.
    running = true;

    Pose2d currentPose = drivetrain.getPose();

    // use last values of filter
    double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());

    Transform2d difference = drivetrain.getPose().minus(targetPose);
    if (Math.abs(difference.getX()) < 0.0762) {
      if (difference.getY() < 0.05 && difference.getY() > 0) {
        yVelocity -= 0.1;
      } else if (difference.getY() > -0.05 && difference.getY() < 0) {
        yVelocity += 0.1;
      }
    }

    int allianceMultiplier = Field2d.getInstance().getAlliance() == Alliance.Blue ? 1 : 1;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            allianceMultiplier * xVelocity, allianceMultiplier * yVelocity, thetaVelocity);

    drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, drivetrain.getRotation()
            /*
             * isFlipped
             * ? drivetrain.getRotation().plus(new Rotation2d(Math.PI))
             * : drivetrain.getRotation())
             */
            ));
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). This command is considered finished if the
   * move-to-pose feature is disabled on the drivetrain subsystem or if the timeout has elapsed or
   * if all the PID controllers are at their goal.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    Transform2d difference = drivetrain.getPose().minus(targetPose);
    Logger.recordOutput("DriveToPose/difference", difference);

    boolean atGoal =
        Math.abs(difference.getX()) < targetTolerance.getX()
            && Math.abs(difference.getY()) < targetTolerance.getY()
            && Math.abs(difference.getRotation().getRadians())
                < targetTolerance.getRotation().getRadians();
    Logger.recordOutput("Swerve Aligned", atGoal);

    // check that running is true (i.e., the calculate method has been invoked on the PID
    // controllers) and that each of the controllers is at their goal. This is important since
    // these
    // controllers will return true for atGoal if the calculate method has not yet been invoked.
    return this.timer.hasElapsed(timeout) || atGoal;
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    running = false;
  }
}
