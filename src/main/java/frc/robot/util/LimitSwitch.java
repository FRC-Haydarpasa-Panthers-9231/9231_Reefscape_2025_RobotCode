package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LimitSwitch extends SubsystemBase {
  private final DigitalInput limitSwitch;

  public LimitSwitch(int port) {
    limitSwitch = new DigitalInput(port);
  }

  public boolean isTrue() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Limit Switch", limitSwitch.get());
  }
}

/**
 * gürültü varsa private boolean lastSensorState = false; private int debounceCounter = 0; private
 * static final int DEBOUNCE_THRESHOLD = 5; // 5 cycle bekleyin @Override public void periodic() {
 * boolean currentSensorState = photoelectricSensor.get();
 *
 * <p>// Debouncing if (currentSensorState != lastSensorState) { debounceCounter++; } else {
 * debounceCounter = 0; }
 *
 * <p>if (debounceCounter >= DEBOUNCE_THRESHOLD) { lastSensorState = currentSensorState;
 * debounceCounter = 0;
 *
 * <p>// Işın kırıldıysa bir işlem yap if (currentSensorState) { resetElevatorPosition(); } } }
 */
