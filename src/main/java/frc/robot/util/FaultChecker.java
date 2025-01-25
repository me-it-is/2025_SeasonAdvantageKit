package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.Elastic.Notification;

public class FaultChecker {
    public List<Fault> faults;
    public String subsystemName;

    public FaultChecker(String subsytemName) {
        this.subsystemName = subsytemName;
    }

    public class Fault {
        String faultName;
        Function<Boolean, Boolean> functionToCheckFault;
        NotificationLevel level = NotificationLevel.WARNING;
        boolean hasFault;

        public Fault(Function<Boolean, Boolean> functionToCheckFault) {
            this.functionToCheckFault = functionToCheckFault;
            faultName = functionToCheckFault.toString().replace("getFault_", "");
        }

        public Fault(Function<Boolean, Boolean> functionToCheckFault, String name) {
            this.functionToCheckFault = functionToCheckFault;
            faultName = name;
        }

        public Fault(Function<Boolean, Boolean> functionToCheckFault, NotificationLevel notificationLevel) {
            this.functionToCheckFault = functionToCheckFault;
            faultName = functionToCheckFault.toString().replace("getFault_", "");
            level = notificationLevel;
        }

        public Fault(Function<Boolean, Boolean> functionToCheckFault, String name, NotificationLevel notificationLevel) {
            this.functionToCheckFault = functionToCheckFault;
            faultName = name;
            level = notificationLevel;
        }
        
        public void updateFault() {
            hasFault = functionToCheckFault.apply(true);
        }
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
