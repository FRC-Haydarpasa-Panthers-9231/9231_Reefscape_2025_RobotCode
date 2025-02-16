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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorRoller.IO_ElevatorRollerReal;
import frc.robot.subsystems.ElevatorRoller.IO_ElevatorRollerSim;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatoRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.IO_ElevatorReal;
import frc.robot.subsystems.elevator.IO_ElevatorSim;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.led.LedConstants;
import frc.robot.subsystems.led.SUB_LED;
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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
    private final SUB_LED leds;

    private double speedRate = 1;
    // Controller
    private final CommandXboxController driverController =
        new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController =
        new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController debugController =
        new CommandXboxController(Constants.OperatorConstants.kPracticeControllerPort);

    private final Alert driverDisconnected =
        new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
    private final Alert operatorDisconnected =
        new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);

    private final Alert debugControllerDisconnected =
        new Alert("Debug controller disconnected (port 2).", AlertType.kInfo);

    private final LoggedNetworkNumber endgameAlert1 =
        new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
    private final LoggedNetworkNumber endgameAlert2 =
        new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
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
                elevatorRoller = new SUB_ElevatoRoller(new IO_ElevatorRollerReal());
                vision =
                    new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight("limelight", drive::getRotation));
                processorPivot = new SUB_ProcessorPivot(new IO_ProcessorPivotReal());
                processorRoller = new SUB_ProcessorRoller(new IO_ProcessorRollerReal());
                elevator = new SUB_Elevator(new IO_ElevatorReal());
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
                elevatorRoller = new SUB_ElevatoRoller(new IO_ElevatorRollerSim());
                processorPivot = new SUB_ProcessorPivot(new IO_ProcessorPivotSim());
                processorRoller = new SUB_ProcessorRoller(new IO_ProcessorRollerSim());
                vision =
                    new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(
                            "photonCamera", VisionConstants.robotToCamera0, drive::getPose));
                elevator = new SUB_Elevator(new IO_ElevatorSim());

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
                elevatorRoller = new SUB_ElevatoRoller(new IO_ElevatorRollerSim());
                processorPivot = new SUB_ProcessorPivot(new IO_ProcessorPivotSim());
                processorRoller = new SUB_ProcessorRoller(new IO_ProcessorRollerSim());
                vision =
                    new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                elevator = new SUB_Elevator(new IO_ElevatorSim());

                break;
        }

        leds = new SUB_LED(LedConstants.kLedPort, LedConstants.kLedPort);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive));

        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

        // FIXME: SİGNAL LOGGER EKLE
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

        driverControllerBindings();
        operatorContorllerBindings();
        debugControllerBindings();
        registerNamedCommands();


        // Endgame Alerts
        new Trigger(
            () -> DriverStation.isTeleopEnabled()
                && DriverStation.getMatchTime() > 0
                && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
                    .onTrue(
                        controllerRumbleCommand()
                            .withTimeout(0.5)
                            .beforeStarting(() -> leds.endgameAlert = true)
                            .finallyDo(() -> leds.endgameAlert = false));
        new Trigger(
            () -> DriverStation.isTeleopEnabled()
                && DriverStation.getMatchTime() > 0
                && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
                    .onTrue(
                        controllerRumbleCommand()
                            .withTimeout(0.2)
                            .andThen(Commands.waitSeconds(0.1))
                            .repeatedly()
                            .withTimeout(0.9)
                            .beforeStarting(() -> leds.endgameAlert = true)
                            .finallyDo(() -> leds.endgameAlert = false)); // Rumble three times
    }

    private void driverControllerBindings()
    {
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

        driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        // Reset gyro to 0° when B button is pressed

        driverController
            .b()
            .onTrue(
                Commands.runOnce(
                    () -> drive.setPose(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                    .ignoringDisable(true));
        driverController
            .y()
            .onTrue(new InstantCommand(() -> speedRate = 0.5))
            .onFalse(new InstantCommand(() -> speedRate = 1));

        // Driver Left Bumper: Face Nearest Reef Face
        driverController
            .leftBumper()
            .whileTrue(
                joystickDriveAtAngle(
                    () -> FieldConstants.getNearestReefFace(drive.getPose())
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
    }

    private void operatorContorllerBindings()
    {

        operatorController
            .leftTrigger()
            .whileTrue(Commands.run(() -> elevator.setSpeed(0.3), elevator))
            .onFalse(Commands.runOnce(() -> elevator.setSpeed(0), elevator));

        operatorController
            .leftBumper()
            .whileTrue(Commands.run(() -> elevator.setSpeed(-0.3), elevator))
            .onFalse(Commands.runOnce(() -> elevator.setSpeed(0), elevator));

        operatorController
            .rightTrigger()
            .whileTrue(Commands.run(() -> processorPivot.setSpeed(0.5), processorPivot))
            .onFalse(Commands.runOnce(() -> processorPivot.setSpeed(0), processorPivot));

        operatorController
            .rightBumper()
            .whileTrue(Commands.run(() -> processorPivot.setSpeed(-0.5), processorPivot))
            .onFalse(Commands.runOnce(() -> processorPivot.setSpeed(0), processorPivot));

        operatorController
            .x()
            .whileTrue(Commands.run(() -> elevatorRoller.setSpeed(0.3), elevatorRoller))
            .onFalse(Commands.runOnce(() -> elevatorRoller.setSpeed(0), elevatorRoller));

        operatorController
            .y()
            .whileTrue(Commands.run(() -> processorRoller.setSpeed(0.3), processorRoller))
            .onFalse(Commands.runOnce(() -> processorRoller.setSpeed(0), processorRoller));

        operatorController
            .b()
            .whileTrue(Commands.run(() -> elevatorRoller.setSpeed(-0.3), elevatorRoller))
            .onFalse(Commands.runOnce(() -> elevatorRoller.setSpeed(0), elevatorRoller));
    }

    private void debugControllerBindings()
    {
        // elevator sysID routines
        debugController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));

        debugController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        debugController.y().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        debugController.a().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        debugController.b().whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        debugController.x().whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        /*
         *
         *
         * driverController
         * .x()
         * .onTrue(Commands.run(() -> elevator.setElevatorVoltage(3), elevator))
         * .onFalse(Commands.run(() -> elevator.setElevatorVoltage(0), elevator));
         *
         * driverController
         * .b()
         * .onTrue(Commands.run(() -> elevator.setElevatorVoltage(-3), elevator))
         * .onFalse(Commands.run(() -> elevator.setElevatorVoltage(0), elevator));
         *
         */

    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY() * speedRate,
            () -> -driverController.getLeftX() * speedRate,
            () -> -driverController.getRightX() * speedRate);
    }

    private Command joystickDriveAtAngle(Supplier<Rotation2d> angle)
    {
        return DriveCommands.joystickDriveAtAngle(
            drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), angle);
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickApproach(drive, () -> -driverController.getLeftY(),
            approachPose);
    }

    /** Register Named commands for use in PathPlanner */
    private void registerNamedCommands()
    {

        // asansör hareket komutu
        NamedCommands.registerCommand(
            "elevatorZeroPose", Commands.run(() -> elevator.runPositonRads(speedRate)));
        NamedCommands.registerCommand("L1", Commands.run(() -> elevator.runPositonRads(speedRate)));
        NamedCommands.registerCommand("L2", Commands.run(() -> elevator.runPositonRads(speedRate)));
        NamedCommands.registerCommand("L3", Commands.run(() -> elevator.runPositonRads(speedRate)));
        NamedCommands.registerCommand("L3", Commands.run(() -> elevator.runPositonRads(speedRate)));

        // asansör roller komutu
        NamedCommands.registerCommand(
            "elavatorRollerOuttake",
            Commands.run(() -> elevatorRoller.setSpeed(0.5), elevatorRoller));
        NamedCommands.registerCommand(
            "elatorRollerStop", Commands.run(() -> elevatorRoller.setSpeed(0), elevatorRoller));

        // processor roller komutu
        NamedCommands.registerCommand(
            "processorRollerStop",
            Commands.run(() -> processorRoller.setSpeed(0), processorRoller));
        NamedCommands.registerCommand(
            "processorRollerFeed",
            Commands.run(() -> processorRoller.setSpeed(0.5), processorRoller));
        NamedCommands.registerCommand(
            "processorRollerOuttake",
            Commands.run(() -> processorRoller.setSpeed(-0.5), processorRoller));
    }

    // Creates controller rumble command
    private Command controllerRumbleCommand()
    {
        return Commands.startEnd(
            () -> {
                driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            },
            () -> {
                driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            });
    }

    // Update dashboard data
    public void updateDashboardOutputs()
    {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    public void updateAlerts()
    {
        // Controller disconnected alerts
        driverDisconnected.set(
            !DriverStation.isJoystickConnected(driverController.getHID().getPort())
                || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
        operatorDisconnected.set(
            !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
                || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
        debugControllerDisconnected.set(!debugController.isConnected());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoChooser.get();
    }

    public CommandXboxController getdriverControllerCommand()
    {
        return driverController;
    }

}
