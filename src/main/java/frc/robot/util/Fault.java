package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.function.Function;

public class Fault {
  String faultName;
  Function<Boolean, StatusSignal<Boolean>> functionToCheckFault;
  NotificationLevel level = NotificationLevel.WARNING;
  boolean hasFault;

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

  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      String name,
      NotificationLevel notificationLevel) {
    this.functionToCheckFault = functionToCheckFault;
    faultName = name;
    level = notificationLevel;
  }

  public void updateFault() {
    hasFault = functionToCheckFault.apply(true).getValue();
  }
}
