package frc.robot.util.BrownoutMonitor;

import static frc.robot.Constants.PDHConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import org.littletonrobotics.junction.Logger;

public class BrownoutMonitor extends SubsystemBase {
  private Drive drive;
  private Climber climber;
  private Elevator elevator;
  private Manipulator manipulator;

  private BrownoutMonitorIO brownoutMonitorIO;
  private BrownoutMonitorIOInputsAutoLogged inputs = new BrownoutMonitorIOInputsAutoLogged();

  public BrownoutMonitor(
      Drive drive,
      Climber climber,
      Elevator elevator,
      Manipulator manipulator,
      BrownoutMonitorIO io) {
    this.brownoutMonitorIO = io;
    this.drive = drive;
    this.climber = climber;
    this.elevator = elevator;
    this.manipulator = manipulator;
  }

  @Override
  public void periodic() {
    brownoutMonitorIO.updateInputs(inputs);
    Logger.processInputs("BrownoutMonitor", inputs);

    if (inputs.totalCurrent.gte(kMaxTotalCurrentDraw)
        || inputs.batteryVoltage.gte(kMaxInputVoltage)) {
      drive.stop();
      climber.stop();
      elevator.stop();
      manipulator.stop();
    }

    // TODO stop specific subsystems for specific PDH channels (0-19 high, 20-23 low)
  }
}
