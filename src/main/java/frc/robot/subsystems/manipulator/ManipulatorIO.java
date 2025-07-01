package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        public Angle manipulatorAngle;
        public double rollerOutput;
        public double pivorOutput;

        public boolean beamBroken;
    }

    public default void setAngle(Angle angle) {}

    public default boolean atAngle() { return false; }

    public default void setRollers() {}

    public default void close() {}
}
