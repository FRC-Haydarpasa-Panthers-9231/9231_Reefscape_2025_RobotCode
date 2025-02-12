package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(
      int maxAttempts, Supplier<StatusCode> command, String errorLocation) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
      if (error.isError()) {
        Elastic.sendNotification(
            new Notification(
                NotificationLevel.ERROR,
                "Kraken Motoru ayarlanırken Hata: " + errorLocation,
                "Kodu ve donanımı kontrol edin!"));
      }
    }
  }
}
