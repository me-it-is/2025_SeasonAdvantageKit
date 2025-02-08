package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.util.datalog.StringLogEntry;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.Objects;
import java.util.function.Function;

public class Fault {
  String faultName;
  Function<Boolean, StatusSignal<Boolean>> functionToCheckFault;
  NotificationLevel level = NotificationLevel.WARNING;
  boolean hasFault;
  boolean hadFault;

  private record FaultInfo(String header, String payload) {
    public FaultInfo {
      Objects.requireNonNull(header);
      Objects.requireNonNull(payload);
    }
  }

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

  public FaultInfo getFaultString(String subsystemName) {
    return new FaultInfo(
        subsystemName + "fault", faultName + " fault" + (hasFault ? "" : " resolved"));
  }

  public void sendNotification(String subsystemName) {
    FaultInfo faultInfo = getFaultString(subsystemName);
    Elastic.sendNotification(new Notification(level, faultInfo.header, faultInfo.payload));
  }

  public void logFault(String subsystemName, StringLogEntry logEntry) {
    FaultInfo faultString = getFaultString(subsystemName);
    logEntry.append(faultString.header + ". " + faultString.payload);
  }
}
