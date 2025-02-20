package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreak extends SubsystemBase {
  // Sensörün baglı oldugu DIO pinini belirleyin (örnegin, DIO 0)
  private final DigitalInput photoelectricSensor = new DigitalInput(Constants.kBeamBreakPort);

  public boolean hasCoral() {
    return photoelectricSensor.get();
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
