package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import monologue.Logged;

public class BrownoutMonitor extends SubsystemBase implements Logged {
  private static double MAX_TOTAL_CURRENT_DRAW = 120.0;
  private static double MAX_HIGH_CHANNEL_DRAW = 40.0;
  private static double MAX_LOW_CHANNEL_DRAW = 15.0;
  public static final double MAX_TEMPERATURE = 85.0;
  public static final double MAX_PDH_INPUT_VOLTAGE = 16.0;
  public static final double BATTERY_VOLTAGE = 12.0;
  public static final double BATTERY_CAPACITY_AH = 18.0;
  public static final double BATTERY_ENERGY_WH = BATTERY_VOLTAGE * BATTERY_CAPACITY_AH;

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
    if (robotPower.getTotalCurrent() > MAX_TOTAL_CURRENT_DRAW) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "too high PDH current draw",
              Double.toString(robotPower.getTotalCurrent())));
      shutdown = true;
    }

    if (robotPower.getVoltage() > MAX_PDH_INPUT_VOLTAGE) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "too high PDH input voltage",
              Double.toString(robotPower.getVoltage())));
      shutdown = true;
    }

    if (robotPower.getTotalEnergy() > BATTERY_ENERGY_WH) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "too fast battery drain",
              Double.toString(robotPower.getTotalEnergy())));
      shutdown = true;
    }

    if (robotPower.getTemperature() > MAX_TEMPERATURE) {
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
