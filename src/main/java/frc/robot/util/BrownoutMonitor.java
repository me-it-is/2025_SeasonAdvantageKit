package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PDHConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import monologue.Logged;

public class BrownoutMonitor extends SubsystemBase implements Logged {
  private PowerDistribution robotPower;
  private Drive drive;
  private Climber climber;
  private Elevator elevator;
  private Manipulator manipulator;

  public BrownoutMonitor(Drive drive, Climber climber, Elevator elevator, Manipulator manipulator) {
    this.robotPower = new PowerDistribution(1, ModuleType.kRev);
    this.drive = drive;
    this.climber = climber;
    this.elevator = elevator;
    this.manipulator = manipulator;
  }

  @Override
  public void periodic() {
    this.log("pdh voltage", robotPower.getVoltage());
    this.log("pdh current", robotPower.getAllCurrents());

    /*if (robotPower.getFaults() != null) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING, "PDH faults", robotPower.getFaults().toString()));
    }*/

    boolean shutdown = false;
    if (Amps.of(robotPower.getTotalCurrent()).gte(PDHConstants.kMaxTotalCurrentDraw)) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "too high PDH current draw",
              Double.toString(robotPower.getTotalCurrent())));
      shutdown = true;
    }

    if (Volts.of(robotPower.getVoltage()).gte(PDHConstants.kMaxInputVoltage)) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "too high PDH input voltage",
              Double.toString(robotPower.getVoltage())));
      shutdown = true;
    }

    if (Joules.of(robotPower.getTotalEnergy()).gte(PDHConstants.kBatteryEnergy)) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "too fast battery drain",
              Double.toString(robotPower.getTotalEnergy())));
      shutdown = true;
    }

    if (Celsius.of(robotPower.getTemperature()).gte(PDHConstants.kMaxTemperature)) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "too high PDH temperature",
              Double.toString(robotPower.getTemperature())));
      shutdown = true;
    }

    if (shutdown) {
      drive.stop();
      climber.stop();
      elevator.stop();
      manipulator.stop();
    }

    // TODO stop specific subsystems for specific PDH channels (0-19 high, 20-23 low)
  }
}
