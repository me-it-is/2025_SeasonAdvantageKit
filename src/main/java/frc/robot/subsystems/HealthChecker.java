package frc.robot.subsystems;

public interface HealthChecker {
  /*
   * The checkHealth function should return a boolean saying if there are any faults in the subsystem
   * it should also if any faults are found create a notifaction on the elastic dashboard with the faults that are detected
   */
  public boolean checkHealth();
}
