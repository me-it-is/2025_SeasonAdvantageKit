package frc.robot.util.BrownoutMonitor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.PowerDistribution;

public class BrownoutMonitorIOReal implements BrownoutMonitorIO {
  private PowerDistribution robotPower;

  public BrownoutMonitorIOReal(PowerDistribution robotPower) {
    this.robotPower = robotPower;
  }

  @Override
  public void updateInputs(BrownoutMonitorIOInputs inputs) {
    inputs.batteryVoltage = Volts.of(robotPower.getVoltage());
    inputs.totalCurrent = Amps.of(robotPower.getTotalCurrent());
  }
}
