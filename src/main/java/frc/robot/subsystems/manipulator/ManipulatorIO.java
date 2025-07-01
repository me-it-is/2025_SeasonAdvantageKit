package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
  @AutoLog
  public static class ManipulatorIOInputs {
    public Angle manipulatorAngle;
    public double rollerOutput;
    public double pivotOutput;

    public boolean beamBroken;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setAngle(Angle angle) {}

  public default void setRollers(double speed) {}

  public default void stop() {}

  public default void close() {}
}
