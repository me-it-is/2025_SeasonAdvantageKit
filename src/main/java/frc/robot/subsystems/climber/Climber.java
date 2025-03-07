package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import monologue.Logged;

public class Climber extends SubsystemBase implements AutoCloseable, Logged {

  private SparkMax motorController;
  private DigitalInput limSwitchUpper;
  private DigitalInput limSwitchLower;
  private DigitalInput lineBreakSensor;

  public enum SensorState {
    TOP,
    BOTTOM,
    MID,
    NONE
  }

  private SensorState curState = SensorState.NONE;

  public Climber(
      SparkMax motorController,
      DigitalInput limSwitchUpper,
      DigitalInput limSwitchLower,
      DigitalInput lineBreakSensor) {
    this.motorController = motorController;
    this.limSwitchUpper = limSwitchUpper;
    this.limSwitchLower = limSwitchLower;
    this.lineBreakSensor = lineBreakSensor;
  }

  @Override
  public void periodic() {
    this.log("lim motor up?", limSwitchUpper.get());
    this.log("lim motor low?", limSwitchLower.get());
    this.log("lim motor mid?", lineBreakSensor.get());

    if (isBottomSwitch()) {
      curState = SensorState.BOTTOM;
    } else if (isTopSwitch()) {
      curState = SensorState.TOP;
    } else if (isLineBreakSwitch()) {
      curState = SensorState.MID;
    }
  }

  @Override
  public void close() {
    motorController.close();
    limSwitchUpper.close();
    limSwitchLower.close();
    lineBreakSensor.close();
  }

  public void setMotor(boolean reverse) {
    int dir = reverse ? -1 : 1;
    motorController.set(ClimberConstants.kClimberMotorMult * dir);
  }

  public void stopMotor() {
    motorController.stopMotor();
  }

  public SparkMax getMotor() {
    return motorController;
  }
  // neutral signal SHOULD then default to brake mode
  public void disable() {
    motorController.disable();
  }

  public boolean isTopSwitch() {
    return limSwitchLower.get();
  }

  public boolean isBottomSwitch() {
    return limSwitchUpper.get();
  }

  public boolean isLineBreakSwitch() {
    return lineBreakSensor.get();
  }

  public SensorState getSensorState() {
    return curState;
  }
}
