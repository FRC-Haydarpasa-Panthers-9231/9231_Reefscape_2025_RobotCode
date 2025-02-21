package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Alert;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
  public static boolean sparkStickyFault = false;

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  public static void tryUntilOk(
      SparkBase spark, int maxAttempts, Supplier<REVLibError> command, Alert configAlert) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        configAlert.set(true);

        sparkStickyFault = true;
      }
    }
  }
}
