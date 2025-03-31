package frc.lib.team3015.subsystem.selfcheck;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkFlex;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingSparkFlex implements SelfChecking {
  private final String label;
  private SparkFlex spark;

  public SelfCheckingSparkFlex(String label, SparkFlex spark) {
    this.label = label;
    this.spark = spark;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    ArrayList<SubsystemFault> faults = new ArrayList<>();

    REVLibError err = spark.getLastError();
    if (err != REVLibError.kOk) {
      faults.add(new SubsystemFault(String.format("[%s]: Error: %s", label, err.name())));
    }

    return faults;
  }

  @Override
  public void clearStickyFaults() {
    spark.clearFaults();
  }
}
