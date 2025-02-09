// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorRoller.IO_ElevatorRollerReal;
import frc.robot.subsystems.ElevatorRoller.IO_ElevatorRollerSim;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.IO_ElevatorReal;
import frc.robot.subsystems.elevator.IO_ElevatorSim;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.processor_pivot.IO_ProcessorPivotReal;
import frc.robot.subsystems.processor_pivot.IO_ProcessorPivotSim;
import frc.robot.subsystems.processor_pivot.SUB_ProcessorPivot;
import frc.robot.subsystems.processor_roller.IO_ProcessorRollerReal;
import frc.robot.subsystems.processor_roller.IO_ProcessorRollerSim;
import frc.robot.subsystems.processor_roller.SUB_ProcessorRoller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Elastic;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final SUB_Elevator elevator;
  private final SUB_ElevatoRoller elevatorRoller;
  private final Vision vision;
  private final SUB_ProcessorPivot processorPivot;
  private final SUB_ProcessorRoller processorRoller;
  private final Superstructure superstructure;

  private double speedRate = 1;
  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new SUB_Elevator(new IO_ElevatorReal());
        elevatorRoller = new SUB_ElevatoRoller(new IO_ElevatorRollerReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight", drive::getRotation));
        processorPivot = new SUB_ProcessorPivot(new IO_ProcessorPivotReal());
        processorRoller = new SUB_ProcessorRoller(new IO_ProcessorRollerReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        elevator = new SUB_Elevator(new IO_ElevatorSim());
        elevatorRoller = new SUB_ElevatoRoller(new IO_ElevatorRollerSim());
        processorPivot = new SUB_ProcessorPivot(new IO_ProcessorPivotSim());
        processorRoller = new SUB_ProcessorRoller(new IO_ProcessorRollerSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "limelight", VisionConstants.robotToCamera0, drive::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new SUB_Elevator(new IO_ElevatorSim());
        elevatorRoller = new SUB_ElevatoRoller(new IO_ElevatorRollerSim());
        processorPivot = new SUB_ProcessorPivot(new IO_ProcessorPivotSim());
        processorRoller = new SUB_ProcessorRoller(new IO_ProcessorRollerSim());
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }
    superstructure =
        new Superstructure(drive, elevator, elevatorRoller, processorPivot, processorRoller, this);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(joystickDrive());

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed

    /*
     * driverController
     * .b()
     * .onTrue(
     * Commands.runOnce(
     * () ->
     * drive.setPose(
     * new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
     * drive)
     * .ignoringDisable(true));
     **/

    driverController
        .y()
        .onTrue(new InstantCommand(() -> speedRate = 0.5))
        .onFalse(new InstantCommand(() -> speedRate = 1));

    driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));

    elevator.setDefaultCommand(Commands.run(() -> elevator.stopElevator(), elevator));

    processorRoller.setDefaultCommand(
        Commands.run(() -> processorRoller.stopMotor(), processorRoller));

    driverController
        .x()
        .onTrue(Commands.run(() -> elevator.setElevatorVoltage(3), elevator))
        .onFalse(Commands.run(() -> elevator.setElevatorVoltage(0), elevator));

    driverController
        .b()
        .onTrue(Commands.run(() -> elevator.setElevatorVoltage(-3), elevator))
        .onFalse(Commands.run(() -> elevator.setElevatorVoltage(0), elevator));

    driverController.y().onTrue(Commands.run(() -> elevator.runPosition(10), elevator));

    // Driver Left Bumper: Face Nearest Reef Face
    driverController
        .leftBumper()
        .whileTrue(
            joystickDriveAtAngle(
                () ->
                    FieldConstants.getNearestReefFace(drive.getPose())
                        .getRotation()
                        .rotateBy(Rotation2d.k180deg)));

    // Driver Left Bumper: Face Nearest Reef Face
    driverController
        .leftBumper()
        .whileTrue(
            joystickDriveAtAngle(
                () ->
                    FieldConstants.getNearestReefFace(drive.getPose())
                        .getRotation()
                        .rotateBy(Rotation2d.k180deg)));

    // Driver Left Bumper + Right Stick Right: Approach Nearest Right-Side Reef Branch
    driverController
        .leftBumper()
        .and(driverController.axisGreaterThan(XboxController.Axis.kRightX.value, 0.8))
        .whileTrue(
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));

    // Driver Left Bumper + Right Stick Left: Approach Nearest Left-Side Reef Branch
    driverController
        .leftBumper()
        .and(driverController.axisLessThan(XboxController.Axis.kRightX.value, -0.8))
        .whileTrue(
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));

    // Driver Left Bumper + Right Bumper: Approach Nearest Reef Face
    driverController
        .leftBumper()
        .and(driverController.rightBumper())
        .whileTrue(joystickApproach(() -> FieldConstants.getNearestReefFace(drive.getPose())));

    /*
     * // elevator sysID routines
     * controller.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
     * controller.y().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * controller.a().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * controller.b().whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * controller.x().whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     */
  }

  private Command joystickDrive() {
    return DriveCommands.joystickDrive(
        drive,
        () -> -driverController.getLeftY() * speedRate,
        () -> -driverController.getLeftX() * speedRate,
        () -> -driverController.getRightX() * speedRate);
  }

  private Command joystickDriveAtAngle(Supplier<Rotation2d> angle) {
    return DriveCommands.joystickDriveAtAngle(
        drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), angle);
  }

  private Command joystickApproach(Supplier<Pose2d> approachPose) {
    return DriveCommands.joystickApproach(drive, () -> -driverController.getLeftY(), approachPose);
  }

  public void setRumbleController(double rumbleSpeed) {
    driverController.setRumble(null, rumbleSpeed);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public CommandXboxController getdriverControllerCommand() {
    return driverController;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    Elastic.Notification notification =
        new Elastic.Notification(
            Elastic.Notification.NotificationLevel.ERROR,
            "Kontrolcü Bağlı Değil!",
            "Kontrolcülerin bağlı olup olmadıklarını kontrol edin");

    if (!driverController.isConnected()) {
      Elastic.sendNotification(notification);
    }
  }
}
