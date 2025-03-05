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

  public Climber(
      SparkMax motorController, DigitalInput limSwitchUpper, DigitalInput limSwitchLower) {
    this.motorController = motorController;
    this.limSwitchUpper = limSwitchUpper;
    this.limSwitchLower = limSwitchLower;
  }

  @Override
  public void periodic() {
    this.log("lim motor up?", limSwitchUpper.get());
    this.log("lim motor low?", limSwitchLower.get());
  }

  @Override
  public void close() {
    motorController.close();
    limSwitchUpper.close();
    limSwitchLower.close();
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
}
