package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ManipulatorConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GameState;
import frc.robot.util.RobotMath;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase implements AutoCloseable {
  private final ManipulatorIO manipulatorIO;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private GameState curState = GameState.NONE;
  private Angle setpoint = Constants.reefMap.get(GameState.NONE).angle();
  private Angle error = Rotations.zero();

  public Manipulator(ManipulatorIO io) {
    manipulatorIO = io;
  }

  @Override
  public void periodic() {
    manipulatorIO.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

    this.error = RobotMath.dist(setpoint, inputs.manipulatorAngle);
    Logger.recordOutput("Manipulator/error", error);
    Logger.recordOutput("Manipulator/at setpoint", atSetpoint());
    Logger.recordOutput("Manipulator/setpoint", setpoint);
  }

  private Angle getAngle(GameState state) {
    return Constants.reefMap.get(state).angle();
  }

  public void setAngle(GameState state) {
    this.curState = state;
    boolean reverse = Constants.reefMap.get(state).angle().gt(setpoint);
    // Speed to counteract inertial of the coral that may be in the manipulator
    manipulatorIO.setRollers(kWhilePivotingSpeed * (reverse ? -1 : 1));
    this.setpoint = getAngle(state);
    manipulatorIO.setAngle(setpoint);
  }

  /** Check if pivot is at angle setpoint to some degree of error */
  public boolean atSetpoint() {
    return error.lt(kRotTolerance);
  }

  /** Spin rollers forward or backward at default speed */
  public void spinRollers(boolean forward) {
    int multiplier = forward == true ? 1 : -1;
    double speed = kDefaultRollerSpeed;
    if (curState == GameState.HUMAN_PLAYER_STATION) {
      speed = kHumanPlayerStationSpeed;
    }
    manipulatorIO.setRollers(speed * multiplier);
  }

  public boolean hasCoral() {
    return inputs.beamBroken;
  }

  public void stopRollers() {
    manipulatorIO.setRollers(0.0);
  }

  public void stop() {
    manipulatorIO.stop();
  }

  @Override
  public void close() throws Exception {
    manipulatorIO.close();
  }
}
