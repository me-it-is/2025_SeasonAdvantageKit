package frc.robot.util.BrownoutMonitor;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface BrownoutMonitorIO {
  @AutoLog
  public class BrownoutMonitorIOInputs {
    public Voltage batteryVoltage;
    public Current totalCurrent;
  }

  public default void updateInputs(BrownoutMonitorIOInputs inputs) {}
}
