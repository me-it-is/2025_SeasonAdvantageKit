package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.State;
import frc.robot.util.RobotMath;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase implements AutoCloseable {
  private Angle error;

  private State curState = State.BOTTOM;
  private Angle setpoint;

  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private ClimberIO climberIO;

  public Climber(ClimberIO io) {
    this.setpoint = stateMap.get(curState);
    this.climberIO = io;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);

    this.error = this.setpoint.minus(inputs.climberAngle);

    Logger.recordOutput("climber/at setpoint", atSetpoint());
    Logger.recordOutput("climber/setpoint error", error);
    Logger.recordOutput("climber/setpoint", setpoint);
    ;
  }

  @Override
  public void close() {
    climberIO.close();
  }

  public void run(boolean forward) {
    climberIO.setOutput(ClimberConstants.kClimberMotorMult * RobotMath.signBool(forward));
  }

  public void moveToSetpoint(State state) {
    this.setpoint = stateMap.get(state);
    climberIO.setAngle(setpoint);
  }

  public boolean atSetpoint() {
    return this.setpoint.isNear(inputs.climberAngle, setpointTolerance);
  }

  public void stop() {
    System.out.println("stop climber");
    climberIO.stopMotor();
  }
}
