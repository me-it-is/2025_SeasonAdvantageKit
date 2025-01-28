package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.time.Duration;
import java.time.Instant;
import java.util.Optional;
import java.util.function.Function;

public class Fault {
  String faultName;
  Function<Boolean, StatusSignal<Boolean>> functionToCheckFault;
  NotificationLevel level = NotificationLevel.WARNING;
  boolean hasFault;
  Optional<Long> minimumTimeBetweenNotifications = Optional.empty();
  Instant lastNotification = Instant.now();

  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
  }

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      long minimumTimeBetweenNotification) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
    this.minimumTimeBetweenNotifications = Optional.of(minimumTimeBetweenNotification);
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
    this.minimumTimeBetweenNotifications = Optional.of(minimumTimeBetweenNotifications);
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
    this.minimumTimeBetweenNotifications = Optional.of(minimumTimeBetweenNotifacations);
  }

  public void updateFault() {
    hasFault = functionToCheckFault.apply(true).getValue();
  }

  public void sendNoftifacation(String subsystemName) {
    Duration timeSenseLastNotification = Duration.between(lastNotification, Instant.now());
    if (minimumTimeBetweenNotifications.isPresent()) {
      if (timeSenseLastNotification.toSeconds() >= minimumTimeBetweenNotifications.get()) {
        Elastic.sendNotification(
            new Notification(level, subsystemName + "fault", faultName + " fault"));
      }
      lastNotification = Instant.now();
    }
  }
}
