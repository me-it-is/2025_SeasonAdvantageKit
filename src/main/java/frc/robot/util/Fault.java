package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.time.Duration;
import java.time.Instant;
import java.util.function.Function;

public class Fault {
  String faultName;
  Function<Boolean, StatusSignal<Boolean>> functionToCheckFault;
  NotificationLevel level = NotificationLevel.WARNING;
  boolean hasFault;
  long minimumTimeBetweenNotifications = Long.MAX_VALUE;
  Instant lastNotification = Instant.now();
  boolean hasSentNotification = false;

  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
  }

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      long minimumTimeBetweenNotification) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
    this.minimumTimeBetweenNotifications = minimumTimeBetweenNotification;
  }

  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault, String name) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = name;
  }

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      String name,
      long minimumTimeBetweenNotifications) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = name;
    this.minimumTimeBetweenNotifications = minimumTimeBetweenNotifications;
  }

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      NotificationLevel notificationLevel) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
    level = notificationLevel;
  }

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      String name,
      NotificationLevel notificationLevel,
      long minimumTimeBetweenNotifacations) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = name;
    level = notificationLevel;
    this.minimumTimeBetweenNotifications = minimumTimeBetweenNotifacations;
  }

  public void updateFault() {
    hasFault = functionToCheckFault.apply(true).getValue();
  }

  public void sendNoftifacation(String subsystemName) {
    Duration timeSenseLastNotification = Duration.between(lastNotification, Instant.now());
    if (timeSenseLastNotification.toSeconds() >= minimumTimeBetweenNotifications
        || !hasSentNotification) {
      Elastic.sendNotification(
          new Notification(level, subsystemName + "fault", faultName + " fault"));
    }
    lastNotification = Instant.now();
  }
}
