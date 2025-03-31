package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainIO;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.vision.Vision;
import frc.robot.Constants.Mode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.factory.AutonomousCommandFactory;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.operator_interface.DebugConsole;
import frc.robot.operator_interface.DebugConsole2;
import frc.robot.operator_interface.DriverConsole;
import frc.robot.operator_interface.OperatorConsole;
import frc.robot.subsystems.ElevatorRoller.IO_ElevatorRollerReal;
import frc.robot.subsystems.ElevatorRoller.SUB_ElevatorRoller;
import frc.robot.subsystems.elevator.IO_ElevatorReal;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.sensors.CoralSensorIOBeam;
import frc.robot.subsystems.sensors.ElevatorLimitSwitchIOReal;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private DriverConsole driverController;
  private OperatorConsole operatorController;
  private DebugConsole debugController;
  private DebugConsole2 debugController2;

  private RobotConfig config;
  private Drivetrain drivetrain;
  private Alliance lastAlliance = Field2d.getInstance().getAlliance();
  private Vision vision;
  private SUB_Elevator elevator;
  private SUB_ElevatorRoller elevatorRoller;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

  private Alert tuningAlert = new Alert("Tuning mode enabled", AlertType.kInfo);

  private edu.wpi.first.wpilibj.smartdashboard.Field2d m_field =
      new edu.wpi.first.wpilibj.smartdashboard.Field2d();

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */
    createRobotConfig();
    createControllers();
    Field2d.getInstance().populateReefBranchPoseMaps();

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {

      switch (Constants.getRobot()) {
        case ROBOT_DEFAULT, ROBOT_PRACTICE, ROBOT_COMPETITION:
          {
            createCTRESubsystems();
            break;
          }
        case ROBOT_SIMBOT:
          {
            createCTRESimSubsystems();
            break;
          }
        default:
          break;
      }

    } else {
      drivetrain = new Drivetrain(new DrivetrainIO() {});
      /*
      manipulator = new Manipulator(new ManipulatorIO() {});
      climber = new Climber(new ClimberIO() {});
      elevator = new Elevator(new ElevatorIO() {});
       */
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    AutonomousCommandFactory.getInstance().configureAutoCommands(drivetrain, vision);
    configureButtonBindings();

    // Alert when tuning
    if (Constants.TUNING_MODE) {
      this.tuningAlert.set(true);
    }
  }

  /**
   * The RobotConfig subclass object *must* be created before any other objects that use it directly
   * or indirectly. If this isn't done, a null pointer exception will result.
   */
  private void createRobotConfig() {
    switch (Constants.getRobot()) {
      case ROBOT_DEFAULT:
        config = new DefaultRobotConfig();
        break;
      default:
        break;
    }
  }

  private void createCTRESubsystems() {
    drivetrain = new Drivetrain(new DrivetrainIOCTRE());
    elevatorRoller =
        new SUB_ElevatorRoller(
            new IO_ElevatorRollerReal(), new CoralSensorIOBeam(config.getCoralSensorID()));
    elevator =
        new SUB_Elevator(
            new IO_ElevatorReal(),
            new ElevatorLimitSwitchIOReal(config.getElevatorLimitSwitchID()));

    /*
    manipulator = new Manipulator(new ManipulatorIOTalonFX());
    climber = new Climber(new ClimberIOTalonFX());
    elevator = new Elevator(new ElevatorIOTalonFX());
      */
  }

  private void createCTRESimSubsystems() {
    drivetrain = new Drivetrain(new DrivetrainIOCTRE());

    /*
    manipulator = new Manipulator(new ManipulatorIOTalonFX());
    climber = new Climber(new ClimberIOTalonFX());
    elevator = new Elevator(new ElevatorIOTalonFX());
     */
  }

  private void createControllers() {
    driverController = new DriverConsole(config.getDriverControllerPort());
    operatorController = new OperatorConsole(config.getOperatorControllerPort());
    debugController = new DebugConsole(config.getDebugControllerPort());
    debugController2 = new DebugConsole2(config.getDebugControllerPort2());
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    configureDrivetrainCommands();

    Operator.registerCommands(operatorController, elevator);

    // Endgame alerts[]
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                .withTimeout(1));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                    .withTimeout(0.5),
                Commands.waitSeconds(0.25),
                Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                    .withTimeout(0.5)));
  }

  private void configureDrivetrainCommands() {
    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            driverController::getTranslateX,
            driverController::getTranslateY,
            driverController::getRotate));

    // lock rotation to the nearest 180° while driving
    driverController
        .getLock180Button()
        .whileTrue(
            new TeleopSwerve(
                    drivetrain,
                    driverController::getTranslateX,
                    driverController::getTranslateY,
                    () ->
                        (drivetrain.getPose().getRotation().getDegrees() > -90
                                && drivetrain.getPose().getRotation().getDegrees() < 90)
                            ? Rotation2d.fromDegrees(0.0)
                            : Rotation2d.fromDegrees(180.0))
                .withName("lock 180"));

    // field-relative toggle
    driverController
        .getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                    Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                    Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                    drivetrain::getFieldRelative)
                .withName("toggle field relative"));

    // slow-mode toggle
    driverController
        .getTranslationSlowModeButton()
        .onTrue(
            Commands.runOnce(drivetrain::enableTranslationSlowMode, drivetrain)
                .withName("enable translation slow mode"));
    driverController
        .getTranslationSlowModeButton()
        .onFalse(
            Commands.runOnce(drivetrain::disableTranslationSlowMode, drivetrain)
                .withName("disable translation slow mode"));
    driverController
        .getRotationSlowModeButton()
        .onTrue(
            Commands.runOnce(drivetrain::enableRotationSlowMode, drivetrain)
                .withName("enable rotation slow mode"));
    driverController
        .getRotationSlowModeButton()
        .onFalse(
            Commands.runOnce(drivetrain::disableRotationSlowMode, drivetrain)
                .withName("disable rotation slow mode"));

    // reset gyro to 0 degrees
    driverController
        .getResetGyroButton()
        .onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain).withName("zero gyro"));

    // x-stance
    driverController
        .getXStanceButton()
        .whileTrue(Commands.run(drivetrain::holdXstance, drivetrain).withName("hold x-stance"));

    // print pose to console for field calibration
    // format the string so that it shows how to make the pose2d object given our current x
    // (double), current y (double), and current rotation (Rotation2d)
    driverController
        .getCurrentPoseButton()
        .onTrue(
            Commands.runOnce(
                    () ->
                        System.out.println(
                            "new Pose2d("
                                + drivetrain.getPose().getTranslation().getX()
                                + ", "
                                + drivetrain.getPose().getTranslation().getY()
                                + ", Rotation2d.fromDegrees("
                                + drivetrain.getPose().getRotation().getDegrees()
                                + "));"))
                .ignoringDisable(true)
                .withName("print current pose"));
  }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() != lastAlliance) {
      this.lastAlliance = alliance.get();
      this.drivetrain.updateAlliance(this.lastAlliance);
      Field2d.getInstance().updateAlliance(this.lastAlliance);
    }
  }

  public void periodic() {
    // add robot-wide periodic code here

    // update LEDs so that they turn yellow. maybe use our reef relative difference, and if it is
    // less than 6
    // but outside of our normal tolerance, turn yellow until we are within tolerance

  }

  public void autonomousInit() {
    // add robot-wide code here that will be executed when autonomous starts
  }

  public void teleopInit() {
    // check if the alliance color has changed based on the FMS data; if the robot power cycled
    // during a match, this would be the first opportunity to check the alliance color based on FMS
    // data.
    this.checkAllianceColor();
  }
  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   */
  private void constructField() {
    frc.robot.Field2d.getInstance().setRegions(new Region2d[] {});
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void logField() {
    SmartDashboard.putData("Field", m_field);
  }

  public void logRobotFieldPosition() {
    m_field.setRobotPose(drivetrain.getPose());
  }

  public void logAutonomousPath() {
    String autoName = "";
    String newAutoName;
    List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();
    newAutoName = (AutonomousCommandFactory.getInstance().getAutonomousCommand()).getName();
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
