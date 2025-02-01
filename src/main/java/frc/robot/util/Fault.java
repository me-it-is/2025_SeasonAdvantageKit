package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.function.Function;

public class Fault {
  String faultName;
  Function<Boolean, StatusSignal<Boolean>> functionToCheckFault;
  NotificationLevel level = NotificationLevel.WARNING;
  boolean hasFault;
  boolean hadFault;

  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
  }

  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault, String name) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = name;
  }

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      NotificationLevel notificationLevel) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = functionToCheckFault.toString().replace("getFault_", "");
    level = notificationLevel;
  }

  /** Updates whether the fault is active and the state in the last call */
  public void updateFault() {
    hadFault = hasFault;
    hasFault = functionToCheckFault.apply(true).getValue();
  }

  public void sendNoftifacation(String subsystemName) {
    Elastic.sendNotification(
        new Notification(
            level, subsystemName + "fault", faultName + " fault" + (hasFault ? "" : " resolved")));
  }
}
