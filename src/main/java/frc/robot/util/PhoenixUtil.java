package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Alert;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command, Alert configAlert) {

    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
      if (error.isError()) {
        configAlert.set(true);
      }
    }
  }

  /**
   * Checks the specified status code and sets the specified alert to the specified message if the
   * status code is not OK.
   *
   * @param statusCode status code to check
   * @param message message to set in the alert if the status code is not OK
   * @param alert alert to set if the status code is not OK
   */
  public static void checkError(StatusCode statusCode, String message, Alert alert) {
    if (statusCode != StatusCode.OK) {
      alert.setText(message + " " + statusCode);
      alert.set(true);
    }
  }
}
