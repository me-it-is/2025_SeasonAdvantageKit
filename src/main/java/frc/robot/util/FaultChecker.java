package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import frc.robot.util.Elastic.Notification;

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
            if (f.hasFault){
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
            Elastic.sendNotification( new Notification(f.level, subsystemName + "fault" , f.faultName +  " fault"));
        }
    }

    public boolean checkForAnyFaults() {
        return getFaults().size() != 0;
    }
}
