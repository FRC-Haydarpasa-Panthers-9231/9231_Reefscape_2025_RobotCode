package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DebugIntaking;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GetCoral;
import frc.robot.commands.IntakingAlgea;
import frc.robot.commands.IntakingCoral;
import frc.robot.commands.ScoringCoral;
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
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_HEIGHT;
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
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.json.simple.parser.ParseException;
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
  // Elastic dashboard'a gyro'yu göstermek için ayrı olarak oluşturduk.
  private GyroIOPigeon2 pigeon = new GyroIOPigeon2();
  private final Field2d m_field = new Field2d();
  // Robotun kontrolcünün hız degeri. Hız bu deger ile çarpılır. Bu sayede
  // istedigimiz zaman
  // yavaşlatabiliriz.
  private double speedRate = 1;
  // Trigger for algae/coral mode switching
  private boolean coralModeEnabled = true;
  private boolean zeroElevatorModeEnabled = true;

  private Trigger isCoralMode = new Trigger(() -> coralModeEnabled);
  private Trigger isZeroingElevator = new Trigger(() -> zeroElevatorModeEnabled);

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
  // Sayaç verilen degerlere geldiginde controller belli bir süre titreyip
  // sürücüye uyarı verir.
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  // Otonom seçmek için widgetd
  private final LoggedDashboardChooser<Command> autoChooser;

  private boolean hasAlgeaOverride = false;

  /** Robot için sarmalayıcı. Subsystems, OI devices, ve commands içerir. */
  public RobotContainer() {

    // Eger voltaj 6.5 altına düşerse roborio'ya komut gitmez.
    RobotController.setBrownoutVoltage(5.5);

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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = new SUB_Elevator(new IO_ElevatorSim());

        break;
    }
    registerNamedCommands();

    // DriverDashboard'a otonomu seçmek için widget oluşturduk.
    // TODO: Içine default otonomu ayarlamayı unutma
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Choices", AutoBuilder.buildAutoChooser("1 piece center"));
    // Set up SysId routines

    // TODO: RUTINLERI ÇALISTIR
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
                    PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
              } catch (Exception e) {
                pathFileMissingAlert.setText("Could not find the specified path file: Start Point");
                pathFileMissingAlert.set(true);
              }
            },
            drive);

    autoChooser.addOption("Start Point", startPoint);

    driverControllerBindings();
    operatorContorllerBindings();
    debugControllerBindings();
    /**
     * Maç bitimi uyarıları, belirtilen saniyelere geldiginde ledleri endgame alert moduna alır ve
     * kontrolcüleri titreştirir.
     */
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(controllerRumbleCommand().withTimeout(0.5));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9)); // Rumble three times
    // new Trigger(hasAlgeaOverride);

    // TODO : BUNU YAP

    new Trigger(() -> elevator.limitSwitchvalue())
        .onTrue(
            Commands.runOnce(
                () -> {
                  elevator.setEncoderPosition(0);
                }));

    /*
     * new Trigger(() -> processorRoller.hasAlgae())
     * .onTrue(Commands.runOnce(() -> processorRoller.stopMotor(),
     * processorRoller));
     */
  }

  private void driverControllerBindings() {
    // Default sürme komutu
    drive.setDefaultCommand(joystickDrive().withName("Default Drive"));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> new Rotation2d())
                .withName("Lock 0 Degree"));

    /*
     * X tuşuna basıldıgında tekerleklerin hepsini X şekline olur. Bu sayede robotun
     * hareket
     * etmesi çok zor olur. Belli bir pozisyonda sabit kalmak istedigimizde
     * kullanılır.
     */
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive).withName("Stop With X"));

    // Y butonuna basıldıgında gyro'yu resetler.
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true)
                .withName("Reset Gyro"));

    // B tuşuna basıldıgında speedRate'i toggle eder
    driverController
        .b()
        .onTrue(
            Commands.runOnce(() -> speedRate = (speedRate == 1) ? 0.5 : 1)
                .withName("Toggle Speed"));

    // Driver Left Bumper: Face Nearest Reef Face
    driverController
        .leftBumper()
        .whileTrue(
            joystickDriveAtAngle(
                    () ->
                        FieldConstants.getNearestReefFace(drive.getPose())
                            .getRotation()
                            .rotateBy(Rotation2d.k180deg))
                .withName("Face Nearest Reef Face"));

    // Driver Left Bumper + Right Stick Right: Approach Nearest Right-Side Reef
    // Branch
    driverController
        .leftBumper()
        .and(driverController.axisGreaterThan(XboxController.Axis.kRightX.value, 0.4))
        .whileTrue(
            joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT))
                .withName("Approach Nearest Right-Side Reef Branch"));

    // Driver Left Bumper + Right Stick Left: Approach Nearest Left-Side Reef Branch
    driverController
        .leftBumper()
        .and(driverController.axisLessThan(XboxController.Axis.kRightX.value, -0.4))
        .whileTrue(
            joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT))
                .withName("Approach Nearest Left-Side Reef Branch"));

    // Driver Left Bumper + Right Bumper: Approach Nearest Reef Face
    driverController
        .leftBumper()
        .and(driverController.rightBumper())
        .whileTrue(
            joystickApproach(() -> FieldConstants.getNearestReefFace(drive.getPose()))
                .withName("Approach Nearest Reef Face"));

    driverController
        .axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.8)
        .and(driverController.rightBumper())
        .whileTrue(
            DriveCommands.pathfindingCommandToPose(1.25, 7.16, -57.17, drive)
                .withName("Drive to Top Feeder"));

    driverController
        .axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.8)
        .and(driverController.rightTrigger())
        .whileTrue(
            DriveCommands.pathfindingCommandToPose(1.13, 0.97, 52.76, drive)
                .withName("Drive to Bottom Feeder"));
    driverController
        .pov(0)
        .whileTrue(
            DriveCommands.pathfindingCommandToPose(5.98, 0.59, -90.30, drive)
                .withName("Drive to processor"));
  }

  private void operatorContorllerBindings() {

    operatorController
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.6)
        .whileTrue(Commands.run(() -> elevator.setSpeed(-0.4)))
        .onFalse(Commands.runOnce(() -> elevator.setSpeed(0)));

    operatorController
        .rightBumper()
        .whileTrue(Commands.runOnce(() -> elevatorRoller.setSpeed(0.4)))
        .onFalse(Commands.runOnce(() -> elevatorRoller.stopmotors(), elevatorRoller));

    operatorController.leftBumper().onTrue(new DebugIntaking(elevatorRoller).withTimeout(6));

    operatorController
        .a()
        .and(isCoralMode)
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.setPosition(
                        ElevatorConstants.ELEVATOR_HEIGHT.ZERO_HEIGHT.getPositionRads()),
                elevator));

    operatorController
        .b()
        .and(isCoralMode)
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.setPosition(
                        ElevatorConstants.ELEVATOR_HEIGHT.CORAL_L2_HEIGHT.getPositionRads()),
                elevator));
    operatorController
        .x()
        .and(isCoralMode)
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.setPosition(
                        ElevatorConstants.ELEVATOR_HEIGHT.CORAL_L3_HEIGHT.getPositionRads()),
                elevator));

    operatorController
        .y()
        .and(isCoralMode)
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.setPosition(
                        ElevatorConstants.ELEVATOR_HEIGHT.CORAL_L4_HEIGHT.getPositionRads()),
                elevator));
    /*
     * operatorController
     * .a()
     * .and(isCoralMode.negate())
     * .onTrue(new IntakingAlgaeGround(elevator, processorRoller, processorPivot));
     *
     * operatorController
     * .b()
     * .and(isCoralMode.negate())
     * .onTrue(new CleaningL2Reef(elevator, processorRoller, processorPivot));
     * operatorController
     * .x()
     * .and(isCoralMode.negate())
     * .onTrue(new CleaningL3Reef(elevator, processorRoller, processorPivot));
     * operatorController
     * .y()
     * .and(isCoralMode.negate())
     *
     * .onTrue(new ScoringAlgea(processorPivot, processorRoller, elevator));
     */

    operatorController.pov(90).onTrue(new IntakingAlgea(processorRoller).withTimeout(3));
    operatorController
        .pov(270)
        .whileTrue(Commands.run(() -> processorRoller.setSpeed(-0.3), processorRoller))
        .onFalse(Commands.runOnce(() -> processorRoller.setSpeed(0), processorRoller));
    operatorController
        .pov(180)
        .whileTrue(Commands.runOnce(() -> processorPivot.setPosition(105), processorPivot))
        .onFalse(Commands.runOnce(() -> processorPivot.stopMotor(), processorPivot));
    // Driver Right Bumper: Toggle between Coral and Algae Modes.
    // Make sure the Approach nearest reef face does not mess with this
    operatorController.pov(0).onTrue(setCoralAlgaeModeCommand());
  }

  private void debugControllerBindings() {
    /*
     * debugController
     * .a()
     * .onTrue(
     * Commands.runOnce(
     * () -> elevator.setPosition(
     * ElevatorConstants.ELEVATOR_HEIGHT.ZERO_HEIGHT.getPositionRads()),
     * elevator));
     *
     * debugController
     * .b()
     * .onTrue(
     * Commands.runOnce(
     * () -> elevator.setPosition(
     * ElevatorConstants.ELEVATOR_HEIGHT.CORAL_L2_HEIGHT.getPositionRads()),
     * elevator)
     * .withName("Elevator position L2"));
     *
     * debugController
     * .x()
     * .onTrue(
     * Commands.runOnce(
     * () -> elevator.setPosition(
     * ElevatorConstants.ELEVATOR_HEIGHT.CORAL_L3_HEIGHT.getPositionRads()),
     * elevator)
     * .withName("Elevator position L3"));
     * debugController
     * .y()
     * .onTrue(
     * Commands.runOnce(
     * () -> elevator.setPosition(
     * ElevatorConstants.ELEVATOR_HEIGHT.CORAL_L4_HEIGHT.getPositionRads()),
     * elevator)
     * .withName("Elevator position L4"));
     *
     * debugController
     * .axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.8)
     * .whileTrue(Commands.run(() -> elevator.setElevatorDebugVoltage(true),
     * elevator))
     * .onFalse(Commands.runOnce(() -> elevator.stopElevator(), elevator));
     *
     * debugController
     * .axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.8)
     * .whileTrue(Commands.run(() -> elevator.setElevatorDebugVoltage(false),
     * elevator))
     * .onFalse(Commands.runOnce(() -> elevator.stopElevator(), elevator));
     *
     * debugController
     * .rightBumper()
     * .whileTrue(
     * Commands.run(() -> elevatorRoller.setSpeed(0.3), elevatorRoller)
     * .withName("Elevator Rollerlar çalıştı"))
     * .onFalse(Commands.runOnce(() -> elevatorRoller.setSpeed(0), elevatorRoller));
     * debugController
     * .leftBumper()
     * .whileTrue(
     * Commands.run(() -> elevatorRoller.setSpeed(-0.5), elevatorRoller)
     * .withName("Elevator Rollerlar çalıştı"))
     * .onFalse(Commands.runOnce(() -> elevatorRoller.setSpeed(0), elevatorRoller));
     *
     * debugController.pov(0).onTrue(new
     * DebugIntaking(elevatorRoller).withTimeout(6));
     * debugController.pov(90)
     * .whileTrue(Commands.run(() -> processorRoller.setSpeed(-0.5),
     * processorRoller))
     * .onFalse(Commands.runOnce(() -> processorRoller.stopMotor(),
     * processorRoller));
     * debugController.pov(270)
     * .whileTrue(Commands.run(() -> processorRoller.setSpeed(-0.5),
     * processorRoller))
     * .onFalse(Commands.runOnce(() -> processorRoller.stopMotor(),
     * processorRoller));
     *
     * // debugController.pov(0).whileTrue(new IntakingCoral(elevatorRoller));
     */
    /*
     * ELEVATOR SYSID TESTS
     * debugController.povUp().onTrue(Commands.runOnce(() -> SignalLogger.start()));
     * debugController.povDown().onTrue(Commands.runOnce(() ->
     * SignalLogger.stop()));
     * debugController.y().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.
     * Direction.kForward))
     * ;
     * debugController.a().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.
     * Direction.kReverse))
     * ;
     * debugController.b().whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.
     * kForward));
     * debugController.x().whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.
     * kReverse));
     */

    debugController.y().onTrue(Commands.run(() -> processorPivot.setPosition(42), processorPivot));
    // .onFalse(Commands.runOnce(() -> processorPivot.stopMotor()));
    debugController
        .b()
        .onTrue(Commands.run(() -> processorPivot.setPosition(35), processorPivot, processorPivot));
    // .onFalse(Commands.runOnce(() -> processorPivot.stopMotor()));
    debugController
        .x()
        .onTrue(Commands.run(() -> processorPivot.setPosition(25), processorPivot, processorPivot));
    // .onFalse(Commands.runOnce(() -> processorPivot.stopMotor()));
    debugController
        .a()
        .onTrue(Commands.run(() -> processorPivot.setPosition(-120), processorPivot));
    // .onFalse(Commands.runOnce(() -> processorPivot.stopMotor()));
    ;

    debugController
        .pov(0)
        .whileTrue(Commands.runOnce(() -> processorRoller.setSpeed(0.3), processorRoller))
        .onFalse(Commands.runOnce(() -> processorRoller.stopMotor(), processorRoller));
    debugController
        .pov(180)
        .whileTrue(Commands.runOnce(() -> processorRoller.setSpeed(-0.3), processorRoller))
        .onFalse(Commands.runOnce(() -> processorRoller.stopMotor(), processorRoller));

    // debugController.x().whileTrue(
    // Commands.run(() -> processorPivot.setPosition(), processorPivot,
    // processorPivot));

    // ARM SYSID TESTS
    /*
     * debugController.y().whileTrue(processorPivot.sysIdQuasistatic(SysIdRoutine.
     * Direction.
     * kForward));
     * debugController.a().whileTrue(processorPivot.sysIdQuasistatic(SysIdRoutine.
     * Direction.
     * kReverse));
     * debugController.b().whileTrue(processorPivot.sysIdDynamic(SysIdRoutine.
     * Direction.kForward
     * ));
     * debugController.x().whileTrue(processorPivot.sysIdDynamic(SysIdRoutine.
     * Direction.kReverse
     * ));
     */
  }

  private Command joystickDrive() {

    return DriveCommands.joystickDrive(
        drive,
        () -> driverController.getLeftY() * speedRate,
        () -> driverController.getLeftX() * speedRate,
        () -> -driverController.getRightX() * speedRate);
  }

  private Command joystickDriveAtAngle(Supplier<Rotation2d> angle) {
    return DriveCommands.joystickDriveAtAngle(
        drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), angle);
  }

  private Command joystickApproach(Supplier<Pose2d> approachPose) {

    return DriveCommands.joystickApproach(drive, () -> -driverController.getLeftY(), approachPose);
  }

  /** Register Named commands for use in PathPlanner */
  private void registerNamedCommands() {

    // asansör hareket komutu
    NamedCommands.registerCommand(
        "elevatorZeroPose",
        Commands.runOnce(() -> elevator.setPosition(ELEVATOR_HEIGHT.ZERO_HEIGHT.getPositionRads()))
            .withName("Elevator Zero Pose"));

    NamedCommands.registerCommand(
        "L1",
        new ScoringCoral(elevator, elevatorRoller, ELEVATOR_HEIGHT.CORAL_L1_HEIGHT)
            .withName("Scoring Coral L1"));
    NamedCommands.registerCommand(
        "L2",
        new ScoringCoral(elevator, elevatorRoller, ELEVATOR_HEIGHT.CORAL_L2_HEIGHT)
            .withName("Scoring Coral L2"));

    NamedCommands.registerCommand(
        "L3",
        new ScoringCoral(elevator, elevatorRoller, ELEVATOR_HEIGHT.CORAL_L3_HEIGHT)
            .withName("Scoring Coral L3"));
    NamedCommands.registerCommand(
        "L4",
        new ScoringCoral(elevator, elevatorRoller, ELEVATOR_HEIGHT.CORAL_L4_HEIGHT)
            .withName("Scoring Coral L4"));

    NamedCommands.registerCommand(
        "getCoral", new IntakingCoral(elevatorRoller).withName("Intaking Coral"));

    NamedCommands.registerCommand(
        "align-right",
        joystickApproach(() -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT))
            .withName("Align Right"));
    NamedCommands.registerCommand(
        "align-center",
        joystickApproach(() -> FieldConstants.getNearestReefFace(drive.getPose()))
            .withName("Align Center"));
    NamedCommands.registerCommand(
        "align-left",
        joystickApproach(() -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT))
            .withName("Align Left"));

    // asansör roller komutu
    NamedCommands.registerCommand(
        "elavatorRollerOuttake", Commands.run(() -> elevatorRoller.setSpeed(0.5), elevatorRoller));
    NamedCommands.registerCommand(
        "elatorRollerStop", Commands.run(() -> elevatorRoller.setSpeed(0), elevatorRoller));
    NamedCommands.registerCommand("getCoral", new GetCoral(elevatorRoller, elevator));

    // processor roller komutu
    NamedCommands.registerCommand(
        "processorRollerStop", Commands.run(() -> processorRoller.setSpeed(0), processorRoller));
    NamedCommands.registerCommand(
        "processorRollerFeed", Commands.run(() -> processorRoller.setSpeed(0.5), processorRoller));
    NamedCommands.registerCommand(
        "processorRollerOuttake",
        Commands.run(() -> processorRoller.setSpeed(-0.5), processorRoller));
  }

  // Creates controller rumble command
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
            () -> {
              driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
              operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            },
            () -> {
              driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
              operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            })
        .withName("Controllers Rumble");
  }

  public Command setCoralAlgaeModeCommand() {
    return Commands.runOnce(
            () -> {
              coralModeEnabled = !coralModeEnabled;
            })
        .withName("Setting Algea-Coral Mode");
  }

  public Command setZeroingElevatorMode() {
    return Commands.runOnce(
            () -> {
              zeroElevatorModeEnabled = !zeroElevatorModeEnabled;
            })
        .withName("Setting Zeroing Elevator Mode");
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
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
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Pigeon'u elastic dashboard'a aktarmak için gerekli fonksiyonu burdan alıcaz. */
  public void addPigeonToDashboard() {
    pigeon.addToSmartDashboard();
  }

  public void logField() {
    SmartDashboard.putData("Field", m_field);
  }

  public void logRobotFieldPosition() {
    m_field.setRobotPose(drive.getPose());
  }

  public void logAutonomousPath() {
    String autoName = "";
    String newAutoName;
    List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();
    newAutoName = (getAutonomousCommand()).getName();
    if (autoName != newAutoName) {
      autoName = newAutoName;
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
        try {
          pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        } catch (IOException a) {
        } catch (ParseException b) {
        } finally {
        }
        ;
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : pathPlannerPaths) {
          poses.addAll(
              path.getAllPathPoints().stream()
                  .map(
                      point ->
                          new Pose2d(
                              point.position.getX(), point.position.getY(), new Rotation2d()))
                  .collect(Collectors.toList()));
        }
        m_field.getObject("path").setPoses(poses);
      }
    }
  }
}
