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

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.led.SUB_LED;
import frc.robot.util.Elastic;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private CANBus canivoreBus;

  private static final double CAN_ERROR_TIME_THRESHOLD = 0.5; // Seconds to disable alert
  private static final double CANIVORE_ERROR_TIME_THRESHOLD = 0.5;

  private boolean autoMessagePrinted;
  private double autoStart;

  private PowerDistribution pdh = new PowerDistribution(19, ModuleType.kRev);
  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private static final double lowBatteryVoltage = 11.3;
  private static final double lowBatteryDisabledTime = 1.5;
  private static final double lowBatteryMinCycleCount = 10;
  private static int lowBatteryCycleCount = 0;
  private final Alert lowBatteryAlert =
      new Alert("Akü voltajı düşük, robotu kapatın ve aküyü değiştirin.", AlertType.kWarning);

  private final Alert canErrorAlert =
      new Alert("CAN hatası tespit edildi, robot kontrol edilemeyebilir.", AlertType.kError);
  private final Alert canivoreErrorAlert =
      new Alert("CANivore hatasi tespit edildi, robot kontrol edilemeyebilir.", AlertType.kError);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging kapasitesi aşıldı, data daha fazla kaydedilmeyecek.", AlertType.kError);

  public Robot() {

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // AdvantageKit logger'ı başlat
    Logger.start();

    // start Elastic Dashboard server
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // DO THIS FIRST
    Pathfinding.setPathfinder(new LocalADStarAK());

    // robot container'ı oluştur.
    robotContainer = new RobotContainer();

    if (!Constants.kIsCompetition) {
      LiveWindow.disableAllTelemetry();
    }
    // PDH log
    SmartDashboard.putData("PDH", pdh);

    // Command schedule'u driver dashoard'a koymak için logladık
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Pigeon'u dashboard'a koymak için logladık
    robotContainer.addPigeonToDashboard();

    disabledTimer.reset();
    canErrorTimer.reset();
    canivoreErrorTimer.reset();
    canInitialErrorTimer.reset();

    // Logging of autonomous paths
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        pose -> Logger.recordOutput("PathFollowing/currentPose", pose));

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("PathFollowing/targetPose", pose));

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("PathFollowing/activePath", poses.toArray(new Pose2d[0])));

    robotContainer.logField();

    this.canivoreBus = new CANBus("*");
    PathfindingCommand.warmupCommand().schedule();
  }

  /** Bu fonksiyon bütün modlarda düzenli olarak çalışır */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    robotContainer.updateAlerts();
    robotContainer.updateDashboardOutputs();

    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    Logger.recordOutput("CANStatus/OffCount", canStatus.busOffCount);
    Logger.recordOutput("CANStatus/TxFullCount", canStatus.txFullCount);
    Logger.recordOutput("CANStatus/ReceiveErrorCount", canStatus.receiveErrorCount);
    Logger.recordOutput("CANStatus/TransmitErrorCount", canStatus.transmitErrorCount);

    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD)
            && canInitialErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD));

    // Log CANivore status
    if (Constants.currentMode == Constants.Mode.REAL) {
      var canivoreStatus = this.canivoreBus.getStatus();
      Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.Status.getName());
      Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.BusUtilization);
      Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.BusOffCount);
      Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.TxFullCount);
      Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.REC);
      Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.TEC);
      if (!canivoreStatus.Status.isOK() || canivoreStatus.REC > 0 || canivoreStatus.TEC > 0) {
        canivoreErrorTimer.restart();
      }
      canivoreErrorAlert.set(
          !canivoreErrorTimer.hasElapsed(CANIVORE_ERROR_TIME_THRESHOLD)
              && canInitialErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD));
    }

    // Düşük pil uyarısı döngü sayısını bir artır.

    lowBatteryCycleCount += 1;

    /**
     * devre dışı kalma zamanlayıcısını sıfırla. Bu, pil düşük olsa bile robotun etkin modda oldugu
     * sürece devre dışı kalma zamanını kontrol etmesini engeller.
     */
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }

    /*
     * Pil voltajını kontrol et. Eger pil voltajı belirlenen düşük voltaj sınırının altındaysa
     * ve devre dışı kalma zamanlayıcısı belirlenen süreyi aşmışsa (lowBatteryDisabledTime),
     * ayrıca düşük pil uyarısı minimum döngü sayısına (lowBatteryMinCycleCount) ulaştıysa,
     * düşük pil uyarısını etkinleştir.
     */
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)
        && lowBatteryCycleCount >= lowBatteryMinCycleCount) {

      // Düşük pil uyarısı aktif hale getirir.
      lowBatteryAlert.set(true);
      // ledleri bataryanın düşük oldugunu gösterecek şekilde yakar.
      SUB_LED.getInstance().lowBatteryAlert = true;
    }

    // Otonom süresini yazdır. Bu sayede otonomda kaç saniye geçtigine göre uzatabiliriz.
    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }
    robotContainer.logRobotFieldPosition();
  }

  @Override
  public void robotInit() {
    // Baglı olan kamerayı dashboard'a koymak için yayınlamaya başlar.
    CameraServer.startAutomaticCapture();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    if (!Constants.debug && Constants.kIsCompetition && !Constants.tuningMode) {
      Elastic.selectTab("Before Match Info");
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    robotContainer.logAutonomousPath();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    /**
     * Otonom dashboard'ı ilk dashboard oldugu için otonom başladıgında otomatik olarak dashboard'ı
     * ayarlar.
     */
    if (!Constants.debug && Constants.kIsCompetition && !Constants.tuningMode) {
      Elastic.selectTab("Autonomous");
    }

    // Otonom zamanlayıcısını başlatır.
    autoStart = Timer.getFPGATimestamp();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // Teleop dashboard'ı 1.tab oldugu için otomatik olarak 1.tab'ı seçer.
    if (!Constants.debug && Constants.kIsCompetition && !Constants.tuningMode) {
      Elastic.selectTab("Teleoperated");
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // Otomatik olarak simulasyonda mavi ittifaka ayarlar
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

    // Gelen joystick baglanmadı uyarılarını kapatır.
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
