package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.RobotMath;
import monologue.Logged;

public class Climber extends SubsystemBase implements AutoCloseable, Logged {

  private SparkMax motorController;

  public enum SensorState {
    TOP,
    BOTTOM,
    MID,
    NONE
  }

  private SensorState curState = SensorState.NONE;

  public Climber(SparkMax motorController) {
    this.motorController = motorController;
  }

  @Override
  public void close() {
    motorController.close();
  }

  public void run(boolean reverse) {
    motorController.set(ClimberConstants.kClimberMotorMult * RobotMath.signBool(reverse));
  }

  public void stop() {
    motorController.stopMotor();
  }

  public SparkMax getMotor() {
    return motorController;
  }
  // neutral signal SHOULD then default to brake mode
  public void disable() {
    motorController.disable();
  }

  public SensorState getSensorState() {
    return curState;
  }
}
