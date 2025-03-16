package frc.robot.util.faultChecker;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.Faults;

public class SparkFaultChecker extends AbstractFaultChecker {
  private Supplier<Faults> faultSupplier;
  public Faults faults;

  public SparkFaultChecker(Supplier<Faults> faultSupplier, String deviceName) {
    super(deviceName);
    this.faultSupplier = faultSupplier;
  }

  public void updateFaults() {
    faults = faultSupplier.get();
    super.updateFaults();
  }
}
