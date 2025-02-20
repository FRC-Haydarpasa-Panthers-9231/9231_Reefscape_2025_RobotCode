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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.CleaningL2Reef;
import frc.robot.commands.CleaningL3Reef;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GetCoral;
import frc.robot.commands.HasCoral;
import frc.robot.commands.IntakingAlgaeGround;
import frc.robot.commands.ScoringAlgea;
import frc.robot.commands.ScoringCoral;
import frc.robot.commands.ZeroElevator;
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
import frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_HEIGHT;
import frc.robot.subsystems.elevator.IO_ElevatorReal;
import frc.robot.subsystems.elevator.IO_ElevatorSim;
import frc.robot.subsystems.elevator.SUB_Elevator;
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
import frc.robot.util.LoggedTunableNumber;
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
    private final SUB_LED leds = SUB_LED.getInstance();
    // Elastic dashboard'a gyro'yu göstermek için ayrı olarak oluşturduk.
    private GyroIOPigeon2 pigeon = new GyroIOPigeon2();

    // Robotun kontrolcünün hız degeri. Hız bu deger ile çarpılır. Bu sayede istedigimiz zaman
    // yavaşlatabiliriz.
    private double speedRate = 1;

    // Trigger for algae/coral mode switching
    private boolean coralModeEnabled = true;
    private Trigger isCoralMode = new Trigger(() -> coralModeEnabled);

    private final IntakingAlgaeGround intakingAlgeaGround;
    private final CleaningL2Reef cleaningl2Reef;
    private final CleaningL3Reef cleaningl3Reef;

    private final ScoringAlgea scoringAlgae;

    private final ScoringCoral scoringCoralL1;
    private final ScoringCoral scoringCoralL2;
    private final ScoringCoral scoringCoralL3;
    private final ScoringCoral scoringCoralL4;

    private final ZeroElevator zeroElevatorCommand;

    /**
     * Kontrolcüler. 1. porttaki driver'ın kontrolcüsü 2.porttaki operator'ün kontrolcüsü 3.porttaki
     * robotu test ederken extra özelliklere ihtiyaç duydugumuz ve tuş kalmadıgı ya da diger
     * kontrolcülerin butonlarını degiştirmek istemedigimiz için kullandıgımız kontrolcü
     */
    private final CommandXboxController driverController =
        new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

    private final CommandXboxController operatorController =
        new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController debugController =
        new CommandXboxController(Constants.OperatorConstants.kPracticeControllerPort);

    // Eger kontrolcüler baglı degilse dashboardda uyarı çıkarır
    private final Alert driverDisconnected =
        new Alert("Driver kontrolcüsünün baglantısı yok!! (port 0).", AlertType.kWarning);
    private final Alert operatorDisconnected =
        new Alert("Operator kontrolcüsünün baglantısı yok! (port 1).", AlertType.kWarning);

    private final Alert debugControllerDisconnected =
        new Alert("Debug kontrolcüsünün baglantısı yok! (port 2).", AlertType.kInfo);

    private Alert pathFileMissingAlert =
        new Alert("Could not find the specified path file.", AlertType.kError);
    // Sayaç verilen degerlere geldiginde controller belli bir süre titreyip sürücüye uyarı verir.
    private final LoggedNetworkNumber endgameAlert1 =
        new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
    private final LoggedNetworkNumber endgameAlert2 =
        new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

    private static final LoggedTunableNumber processorPivotSetpoint =
        new LoggedTunableNumber("ProcessorPivot/Processor Pivot Setpoint", 0);

    private static final LoggedTunableNumber elevatorHeightSetpoint =
        new LoggedTunableNumber("Elevator/Elevator Height Setpoint", 0);
    private static final LoggedTunableNumber elevatorVolts =
        new LoggedTunableNumber("Elevator/Elevator Volts", 2);
    private static final LoggedTunableNumber processorPivotSpeed =
        new LoggedTunableNumber("ProcessorPivot/Processor Pivot speed", 0.1);
    private static final LoggedTunableNumber processorRollerVoltage =
        new LoggedTunableNumber("ProcessorRoller/Processor Roller Voltage", 2);
    private static final LoggedTunableNumber elevatorRollerSpeed =
        new LoggedTunableNumber("ElevatorRoller/Elevator Roller speed", 0.3);

    // Otonom seçmek için widget
    private final LoggedDashboardChooser<Command> autoChooser;

    /** Robot için sarmalayıcı. Subsystems, OI devices, ve commands içerir. */
    public RobotContainer()
    {
        // Eger voltaj 6.5 altına düşerse roborio'ya komut gitmez.
        RobotController.setBrownoutVoltage(6.5);

        switch (Constants.currentMode) {
            case REAL:
                // Gerçek robotta donanım ve io arayüzlerini tanımlar
                drive =
                    new Drive(
                        pigeon,
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
                // Sim robotta donanım ve io arayüzlerini tanımlar
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
                // Tekrar oynatılan robot, bu yüzden io arayüzlerini tanımlamıyoruz
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

        // DriverDashboard'a otonomu seçmek için widget oluşturduk.
        // TODO: Içine default otonomu ayarlamayı unutma
        autoChooser =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("a"));

        // Set up SysId routines
        // TODO: RUTINLERI ÇALISTIR
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive));

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

        autoChooser.addOption("Elevator Characterization", elevator.sysIDCharacterizationRoutine());

        /************
         * Start Point ************
         *
         * useful for initializing the pose of the robot to a known location
         *
         */

        Command startPoint =
            Commands.runOnce(
                () -> {
                    try {
                        drive.setPose(
                            PathPlannerPath.fromPathFile("Start Point")
                                .getStartingDifferentialPose());
                    } catch (Exception e) {
                        pathFileMissingAlert
                            .setText("Could not find the specified path file: Start Point");
                        pathFileMissingAlert.set(true);
                    }
                },
                drive);
        autoChooser.addOption("Start Point", startPoint);

        zeroElevatorCommand = new ZeroElevator(elevator);

        cleaningl3Reef = new CleaningL3Reef(elevator, processorRoller, processorPivot, leds);

        cleaningl2Reef = new CleaningL2Reef(elevator, processorRoller, processorPivot, leds);

        scoringAlgae = new ScoringAlgea(processorPivot, processorRoller, elevator, leds);

        intakingAlgeaGround =
            new IntakingAlgaeGround(elevator, processorRoller, processorPivot, leds);

        scoringCoralL1 =
            new ScoringCoral(elevator, leds, elevatorRoller, ELEVATOR_HEIGHT.CORAL_L1_HEIGHT);
        scoringCoralL2 =
            new ScoringCoral(elevator, leds, elevatorRoller, ELEVATOR_HEIGHT.CORAL_L2_HEIGHT);
        scoringCoralL3 =
            new ScoringCoral(elevator, leds, elevatorRoller, ELEVATOR_HEIGHT.CORAL_L3_HEIGHT);
        scoringCoralL4 =
            new ScoringCoral(elevator, leds, elevatorRoller, ELEVATOR_HEIGHT.CORAL_L4_HEIGHT);
        new Trigger(elevatorRoller::hasCoral).onTrue(new HasCoral(leds));
        new Trigger(processorRoller::hasAlgae).onTrue(new HasCoral(leds));

        driverControllerBindings();
        operatorContorllerBindings();
        debugControllerBindings();
        registerNamedCommands();

        /**
         * Maç bitimi uyarıları, belirtilen saniyelere geldiginde ledleri endgame alert moduna alır
         * ve
         * kontrolcüleri titreştirir.
         */
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
        // Default sürme komutu
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

        /*
         * X tuşuna basıldıgında tekerleklerin hepsini X şekline olur. Bu sayede robotun hareket
         * etmesi çok zor olur. Belli bir pozisyonda sabit kalmak istedigimizde kullanılır.
         */
        driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // B butonuna basıldıgında gyro'yu resetler.
        driverController
            .b()
            .onTrue(
                Commands.runOnce(
                    () -> drive.setPose(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                    .ignoringDisable(true));

        // Y tuşuna basıldıgında speedRate'i toggle eder
        driverController.y()
            .onTrue(new InstantCommand(() -> speedRate = (speedRate == 1) ? 0.5 : 1));

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
            .leftBumper()
            .and(operatorController.rightBumper())
            .onTrue(Commands.run(() -> elevator.setSpeed(0.3)))
            .onFalse(Commands.run(() -> elevator.setSpeed(0)));

        operatorController
            .leftBumper()
            .and(operatorController.rightTrigger())
            .onTrue(Commands.run(() -> elevator.setSpeed(-0.3)))
            .onFalse(Commands.run(() -> elevator.setSpeed(0)));

        operatorController.a().and(isCoralMode).onTrue(scoringCoralL1);
        operatorController.b().and(isCoralMode).onTrue(scoringCoralL2);
        operatorController.x().and(isCoralMode).onTrue(scoringCoralL3);
        operatorController.y().and(isCoralMode).onTrue(scoringCoralL4);

        operatorController.a().and(isCoralMode.negate()).onTrue(intakingAlgeaGround);
        operatorController.b().and(isCoralMode.negate()).onTrue(cleaningl2Reef);
        operatorController.x().and(isCoralMode.negate()).onTrue(cleaningl3Reef);
        operatorController.y().and(isCoralMode.negate()).onTrue(scoringAlgae);

        // Driver Right Bumper: Toggle between Coral and Algae Modes.
        // Make sure the Approach nearest reef face does not mess with this
        operatorController
            .rightBumper()
            .and(operatorController.leftBumper().negate())
            .onTrue(setCoralAlgaeModeCommand());
        operatorController.rightTrigger().onTrue(zeroElevatorCommand);
    }

    private void debugControllerBindings()
    {
        debugController
            .a()
            .onTrue(
                Commands.runOnce(
                    () -> processorPivot.setPosition(processorPivotSetpoint.getAsDouble())));

        debugController
            .b()
            .onTrue(
                Commands
                    .runOnce(() -> elevator.runPositonRads(elevatorHeightSetpoint.getAsDouble())));

        debugController
            .x()
            .onTrue(Commands.run(() -> elevatorRoller.setSpeed(elevatorRollerSpeed.get())))
            .onFalse(Commands.run(() -> elevatorRoller.setSpeed(0)));

        debugController
            .start()
            .onTrue(
                Commands
                    .run(() -> processorRoller.setAlgaeIntakeVoltage(processorRollerVoltage.get())))
            .onFalse(Commands.run(() -> processorRoller.setSpeed(0)));

        debugController
            .back()
            .onTrue(
                Commands.run(
                    () -> processorRoller.setAlgaeIntakeVoltage(-processorRollerVoltage.get())))
            .onFalse(Commands.run(() -> processorRoller.setSpeed(0)));

        debugController
            .rightBumper()
            .whileTrue(
                Commands.run(() -> elevator.setElevatorVoltage(Volts.of(elevatorVolts.get()))))
            .onFalse(Commands.run(() -> elevator.setElevatorVoltage(Volts.of(0))));

        debugController
            .rightTrigger()
            .whileTrue(
                Commands.run(() -> elevator.setElevatorVoltage(Volts.of(-elevatorVolts.get()))))
            .onFalse(Commands.run(() -> elevator.setElevatorVoltage(Volts.of(0))));

        debugController
            .leftBumper()
            .whileTrue(Commands.run(() -> processorPivot.setSpeed(processorPivotSpeed.get())))
            .onFalse(Commands.run(() -> processorPivot.setSpeed(0)));

        debugController
            .leftTrigger()
            .whileTrue(Commands.run(() -> processorPivot.setSpeed(-processorPivotSpeed.get())))
            .onFalse(Commands.run(() -> processorPivot.setSpeed(0)));
    }

    private Command joystickDrive()
    {

        return DriveCommands.joystickDrive(
            drive,
            () -> MathUtil.applyDeadband(
                -driverController.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND)
                * speedRate,
            () -> MathUtil.applyDeadband(
                -driverController.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND)
                * speedRate,
            () -> MathUtil.applyDeadband(
                -driverController.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)
                * speedRate);
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
            "elevatorZeroPose",
            Commands.run(
                () -> elevator
                    .setPosition(Meters.of(ELEVATOR_HEIGHT.ZERO_HEIGHT.getHeightInMeters()))));
        NamedCommands.registerCommand("L1", scoringCoralL1);
        NamedCommands.registerCommand("L2", scoringCoralL2);
        NamedCommands.registerCommand("L3", scoringCoralL3);
        NamedCommands.registerCommand("L4", scoringCoralL4);

        NamedCommands.registerCommand(
            "align-right",
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));
        NamedCommands.registerCommand(
            "align-center",
            joystickApproach(() -> FieldConstants.getNearestReefFace(drive.getPose())));
        NamedCommands.registerCommand(
            "align-left",
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));

        // asansör roller komutu
        NamedCommands.registerCommand(
            "elavatorRollerOuttake",
            Commands.run(() -> elevatorRoller.setSpeed(0.5), elevatorRoller));
        NamedCommands.registerCommand(
            "elatorRollerStop", Commands.run(() -> elevatorRoller.setSpeed(0), elevatorRoller));
        NamedCommands.registerCommand("getCoral", new GetCoral(elevatorRoller, elevator, leds));

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

    public Command setCoralAlgaeModeCommand()
    {
        return Commands.runOnce(
            () -> {
                coralModeEnabled = !coralModeEnabled;
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

    /** Pigeon'u elastic dashboard'a aktarmak için gerekli fonksiyonu burdan alıcaz. */
    public void addPigeonToDashboard()
    {
        pigeon.addToSmartDashboard();
    }
}
