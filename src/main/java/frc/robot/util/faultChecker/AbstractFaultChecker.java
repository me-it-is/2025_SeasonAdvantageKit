package frc.robot.util.faultChecker;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

public abstract class AbstractFaultChecker {
  public List<Fault> warningFaults = new ArrayList<>();
  public List<Fault> errorFaults = new ArrayList<>();

  public String subsystemName;

  private DataLog log = DataLogManager.getLog();
  private StringLogEntry stringLog = new StringLogEntry(log, subsystemName);

  public AbstractFaultChecker(String commponentName) {
    this.subsystemName = commponentName;
    DataLogManager.start();
  }

  public void updateFaults() {
    for (Fault f : warningFaults) {
      f.updateFault();
      if (f.hasFault != f.hadFault) {
        f.logFault(subsystemName, stringLog);
      }
    }
    for (Fault f : errorFaults) {
      f.updateFault();
      if (f.hasFault != f.hadFault) {
        f.sendNotification(subsystemName);
      }
    }
  }

  public List<Fault> getActiveFaults() {
    return Stream.concat(getActiveWarningFaults().stream(), getActiveErrorFaults().stream())
        .toList();
  }

  public List<Fault> getActiveWarningFaults() {
    List<Fault> activeFaults = new ArrayList<>();
    for (Fault f : warningFaults) {
      if (f.hasFault) {
        activeFaults.add(f);
      }
    }
    return activeFaults;
  }

  public List<Fault> getActiveErrorFaults() {
    List<Fault> activeFaults = new ArrayList<>();
    for (Fault f : errorFaults) {
      if (f.hasFault) {
        activeFaults.add(f);
      }
    }
    return activeFaults;
  }

  public void addFault(Fault fault) {
    if (warningFaults != null && fault.level == NotificationLevel.WARNING) {
      this.warningFaults.add(fault);
    }
    if (errorFaults != null && fault.level == NotificationLevel.ERROR) {
      this.errorFaults.add(fault);
    }
  }

  public void addFaults(List<Fault> faults) {
    for (Fault f : faults) {
      addFault(f);
    }
  }

  public boolean hasFault() {
    return getActiveWarningFaults().size() != 0;
  }

  public boolean hasErrorFault() {
    return getActiveErrorFaults().size() != 0;
  }

  /**
   * @return true if no warnings or errors
   */
  public boolean isHealthy() {
    return (!hasFault()) && !(hasErrorFault());
  }

  /**
   * @return true if no warrnings
   */
  public boolean isClean() {
    return !hasFault();
  }

  /**
   * @return true if no errors
   */
  public boolean isAlive() {
    return !hasErrorFault();
  }
}
