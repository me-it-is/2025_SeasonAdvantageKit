package frc.robot.util.faultChecker;

import java.util.ArrayList;
import java.util.List;

public abstract class UnitTestableFaultChecker extends AbstractFaultChecker {
  public List<Fault> loggedFaults;
  public List<Fault> notifiedFaults;

  public UnitTestableFaultChecker(String commponentName) {
    super(commponentName);
    loggedFaults = new ArrayList<Fault>();
    notifiedFaults = new ArrayList<Fault>();
  }

  @Override
  void logFault(Fault fault) {
    loggedFaults.add(fault);
    super.logFault(fault);
  }

  @Override
  void notifyFault(Fault fault) {
    notifiedFaults.add(fault);
    super.notifyFault(fault);
  }

  @Override
  public void updateFaults() {
    loggedFaults = new ArrayList<>();
    notifiedFaults = new ArrayList<>();
    super.updateFaults();
  }
}
