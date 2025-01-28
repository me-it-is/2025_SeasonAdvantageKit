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
  Optional<Long> minimumTimeBetweenNotifacations = Optional.empty();
  Instant lastNotifacation = Instant.now();

  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
  }

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      long minimumTimeBetweenNotifacations) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
    this.minimumTimeBetweenNotifacations = Optional.of(minimumTimeBetweenNotifacations);
  }

  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault, String name) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = name;
  }

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      String name,
      long minimumTimeBetweenNotifacations) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = name;
    this.minimumTimeBetweenNotifacations = Optional.of(minimumTimeBetweenNotifacations);
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
    this.minimumTimeBetweenNotifacations = Optional.of(minimumTimeBetweenNotifacations);
  }

  public void updateFault() {
    hasFault = functionToCheckFault.apply(true).getValue();
  }

  public void sendNoftifacation(String subsystemName) {
    Duration timeSenseLastNotification = Duration.between(lastNotifacation, Instant.now());
    if (minimumTimeBetweenNotifacations.isPresent()) {
      if (timeSenseLastNotification.toSeconds() >= minimumTimeBetweenNotifacations.get()) {
        Elastic.sendNotification(
            new Notification(level, subsystemName + "fault", faultName + " fault"));
      }
      lastNotifacation = Instant.now();
    }
  }
}
