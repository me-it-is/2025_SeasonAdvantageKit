package frc.robot.util.faultChecker;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import java.util.function.Supplier;

public class SparkFaultChecker extends AbstractFaultChecker {
  private Supplier<Faults> faultSupplier;
  public Faults faults;
  /**
   * @param faultSupplier Supplier or faults for the device.
   * @param deviceName Name of the device to check faults on.
   */
  public SparkFaultChecker(Supplier<Faults> faultSupplier, String deviceName) {
    super(deviceName);
    this.faultSupplier = faultSupplier;
  }
  /**
   * @param device SparkMax or SparkFlex to check faults on.
   * @param deviceName Name of the device to check faults on.
   */
  public <U extends SparkBase> SparkFaultChecker(U device, String deviceName) {
    super(deviceName);
    this.faultSupplier = () -> (device.getFaults());
  }

  /**
   * Updates all faults in the fault checker. If any faults have been resolved or stated it will log
   * or report them based on sevarity
   */
  public void updateFaults() {
    faults = faultSupplier.get();
    super.updateFaults();
  }
}
