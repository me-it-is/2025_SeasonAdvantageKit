package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public Angle climberAngle;
    public double motorOutput;
  }

  public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

  public default void setAngle(Angle angle) {}

  public default void setOutput(double output) {}

  public default void stopMotor() {}

  public default void close() {}
}
