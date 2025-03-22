package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.RobotMath;
import frc.robot.util.faultChecker.SparkFaultChecker;
import monologue.Logged;

public class Climber extends SubsystemBase implements AutoCloseable, Logged {

  private SparkMax motorController;
  private SparkMaxConfig config;
  private SparkFaultChecker climberFaultChecker;
  private SparkAbsoluteEncoder encoder;
  private SparkClosedLoopController controller;
  private double encoderPos;
  private double error = 0.0;
  private boolean atSetpoint = false;

  private State curState = State.BOTTOM;
  private Angle setpoint;

  public Climber(SparkMax motorController) {
    this.motorController = motorController;
    this.climberFaultChecker = new SparkFaultChecker(motorController, "Climber");
    this.config = new SparkMaxConfig();
    this.encoder = motorController.getAbsoluteEncoder();
    this.controller = motorController.getClosedLoopController();
    this.setpoint = stateMap.get(curState);
    this.encoderPos = encoder.getPosition();

    config.inverted(true);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
    motorController.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    this.log("climber/appl out", motorController.getAppliedOutput());
    this.log("climber/at setpoint", atSetpoint);
    this.log("climber/setpoint error", error);
    this.log("climber/setpoint", setpoint.in(Rotations));

    encoderPos = encoder.getPosition();
    this.log("climber/encoder pos", encoderPos);

    Angle diff = this.setpoint.minus(Rotations.of(encoderPos));
    this.error = diff.in(Rotations);
    climberFaultChecker.updateFaults();
  }

  @Override
  public void close() {
    motorController.close();
  }

  public void run(boolean forward) {
    motorController.set(ClimberConstants.kClimberMotorMult * RobotMath.signBool(forward));
  }

  public void moveToSetpoint(State state) {
    this.setpoint = stateMap.get(state);
    this.atSetpoint = false;
    controller.setReference(setpoint.in(Rotations), ControlType.kPosition);
  }

  public boolean atSetpoint() {
    this.atSetpoint = Rotations.of(encoder.getPosition()).isNear(setpoint, setpointTolerance);
    return atSetpoint;
  }

  public void stop() {
    System.out.println("stop climber");
    motorController.stopMotor();
  }

  public SparkMax getMotor() {
    return motorController;
  }

  public void disable() {
    motorController.disable();
  }

  public State getSensorState() {
    return curState;
  }
}
