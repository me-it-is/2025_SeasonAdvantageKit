package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    Angle leaderPosition;
    AngularVelocity leaderVelocity;
    Voltage leaderVoltage;
    Current leaderCurrent;
    StatusCode leaderStatus;

    Angle followerPosition;
    AngularVelocity followerVelocity;
    Voltage followerVoltage;
    Current followerCurrent;
    StatusCode followerStatus;
  }

  public default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}

  public default void setSetpoint(Distance hight) {}

  public default void stop() {}

  public default void Close() {}
}
