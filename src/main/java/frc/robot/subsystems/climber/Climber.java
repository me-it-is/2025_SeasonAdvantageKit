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
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.RobotMath;
import frc.robot.util.faultChecker.SparkFaultChecker;

public class Climber extends SubsystemBase implements AutoCloseable {

  protected SparkMax motorController;
  protected SparkMaxConfig config;
  private SparkFaultChecker climberFaultChecker;
  protected SparkAbsoluteEncoder encoder;
  protected SparkClosedLoopController controller;
  private Angle encoderPos;
  private Angle error;

  protected State curState = State.BOTTOM;
  protected Angle setpoint;

  public Climber(SparkMax motorController) {
    this.motorController = motorController;
    this.climberFaultChecker = new SparkFaultChecker(motorController, "Climber");
    this.config = new SparkMaxConfig();
    this.encoder = motorController.getAbsoluteEncoder();
    this.controller = motorController.getClosedLoopController();
    this.setpoint = stateMap.get(curState);
    this.encoderPos = Rotations.of(encoder.getPosition());

    config.inverted(true);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
    motorController.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    encoderPos = Rotations.of(encoder.getPosition());

    this.error = this.setpoint.minus(encoderPos);
    climberFaultChecker.updateFaults();

    DogLog.log("climber/appl out", motorController.getAppliedOutput());
    DogLog.log("climber/at setpoint", atSetpoint);
    DogLog.log("climber/setpoint error", error.in(Rotations));
    DogLog.log("climber/setpoint", setpoint.in(Rotations));
    DogLog.log("climber/encoder pos", encoderPos.in(Rotations));
  }

  @Override
  public void close() {
    motorController.close();
  }

  public void run(boolean reverse) {
    motorController.set(ClimberConstants.kClimberMotorMult * RobotMath.signBool(reverse));
  }

  public Command moveToSetpoint(State state) {
    this.setpoint = stateMap.get(state);
    return run(() -> controller.setReference(setpoint.in(Rotations), ControlType.kPosition))
        .until(this::atSetpoint);
  }

  public boolean atSetpoint() {
    return this.setpoint.isNear(encoderPos, setpointTolerance);
  }

  public Angle getPosition() {
    encoderPos = Rotations.of(encoder.getPosition());
    return encoderPos;
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

  public double getAppliedOutput() {
    return motorController.getAppliedOutput();
  }
}
