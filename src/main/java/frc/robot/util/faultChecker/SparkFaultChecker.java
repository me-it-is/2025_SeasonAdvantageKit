package frc.robot.util.faultChecker;

import com.revrobotics.spark.SparkBase.Faults;
import java.util.function.Supplier;

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
