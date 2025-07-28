package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import frc.robot.util.RobotMath;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase implements AutoCloseable {
  private Distance setpoint = Constants.reefMap.get(GameState.NONE).distance();

  ElevatorIO elevatorIO;
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    elevatorIO = io;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();
    elevatorIO.setSetpoint(setpoint);
  }

  public boolean atSetpoint() {
    return RobotMath.abs(inputs.leaderPosition.minus(heightToAngle(setpoint)))
        .lt(ElevatorConstants.kSetpointTolerance);
  }

  public void sysIdLog(SysIdRoutineLog log) {
    log.motor("Elevator motors")
        .voltage(inputs.leaderVoltage)
        .linearPosition(getElevatorHeight())
        .linearVelocity(getElevatorVelocity());
  }

  public void voltageDrive(Voltage volts) {
    // talonLeader.setControl(new VoltageOut(volts));
  }

  public Current getTotalCurrent() {
    return inputs.followerCurrent.plus(inputs.leaderCurrent);
  }

  public Distance getElevatorHeight() {
    if (inputs.leaderPosition == null) return Meters.zero();
    if (inputs.leaderPosition.lte(Rotations.zero())) return Meters.zero();
    return angleToHeight(inputs.leaderPosition);
  }

  private LinearVelocity getElevatorVelocity() {
    return RobotMath.castToMoreSpecificUnits(
        RobotMath.applyDimesionalAnalysis(ElevatorConstants.kAngularSpan, inputs.leaderVelocity),
        MetersPerSecond.zero());
  }

  protected static Distance angleToHeight(Angle angle) {
    return (Distance) ElevatorConstants.kAngularSpan.timesDivisor(angle);
  }

  protected static Angle heightToAngle(Distance height) {
    return (Angle) ElevatorConstants.kSpanAngle.timesDivisor(height);
  }

  public void zeroElevator() {
    setSetpoint(GameState.NONE);
  }

  public void stop() {
    elevatorIO.stop();
  }

  @Override
  public void close() {
    elevatorIO.Close();
  }
}
