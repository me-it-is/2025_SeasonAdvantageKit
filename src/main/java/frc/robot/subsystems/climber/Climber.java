package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import monologue.Logged;

public class Climber extends SubsystemBase implements AutoCloseable, Logged {

  private SparkMax motorController;
  private DigitalInput m_limSwitchUpper;
  private DigitalInput m_limSwitchLower;

  public Climber() {
    motorController = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);

    motorController.set(0);

    m_limSwitchUpper = new DigitalInput(ClimberConstants.kUpperLimSwitchId);
    m_limSwitchLower = new DigitalInput(ClimberConstants.kUpperLimSwitchId);
  }

  @Override
  public void periodic() {
    this.log("lim motor up?", m_limSwitchUpper.get());
    this.log("lim motor low?", m_limSwitchLower.get());
  }

  @Override
  public void close() {
    motorController.close();
    m_limSwitchUpper.close();
    m_limSwitchLower.close();
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
    return m_limSwitchLower.get();
  }

  public boolean isBottomSwitch() {
    return m_limSwitchUpper.get();
  }
}
