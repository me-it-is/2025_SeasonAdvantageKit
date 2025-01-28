package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

public class FaultChecker {
  public List<Fault> faults;
  public String subsystemName;

  public FaultChecker(String subsytemName) {
    this.subsystemName = subsytemName;
  }

  public void updateFaults() {
    for (Fault f : faults) {
      f.updateFault();
    }
  }

  public List<String> getFaults() {
    List<String> stringFaults = new ArrayList<>();
    for (Fault f : faults) {
      if (f.hasFault) {
        stringFaults.add(f.faultName);
      }
    }
    return stringFaults;
  }

  public void addFault(Fault fault) {
    faults.add(fault);
  }

  public void sendNotifications() {
    for (Fault f : faults) {
      f.sendNoftifacation(subsystemName);
    }
  }

  public boolean checkForAnyFaults() {
    return getFaults().size() != 0;
  }
}
