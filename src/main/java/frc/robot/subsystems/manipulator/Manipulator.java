package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GameState;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.util.faultChecker.SparkFaultChecker;
import monologue.Logged;

public class Manipulator extends SubsystemBase implements Logged, AutoCloseable {
  private SparkMax pivot;
  private SparkMax rollers;
  private SparkMaxConfig pivotConfig;
  private SparkMaxConfig rollerConfig;
  private SparkClosedLoopController pivotController;
  private SparkFaultChecker pivotChecker;
  private SparkFaultChecker rollersChecker;
  private AbsoluteEncoder pivotEncoder;
  private double setpoint = Constants.reefMap.get(GameState.NONE).angle().in(Units.Rotations);
  private double error = 0.0;
  private boolean atSetpoint = false;

  public Manipulator(SparkMax pivot, SparkMax rollers) {
    this.pivot = pivot;
    this.rollers = rollers;
    this.pivotConfig = new SparkMaxConfig();
    this.rollerConfig = new SparkMaxConfig();
    this.pivotEncoder = pivot.getAbsoluteEncoder();
    this.pivotController = pivot.getClosedLoopController();
    this.pivotChecker = new SparkFaultChecker(pivot, "Pivot SparkMax");
    this.rollersChecker = new SparkFaultChecker(rollers, "Rollers SparkMax");

    pivotConfig.inverted(false).smartCurrentLimit(ManipulatorConstants.currentLimit);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
    rollerConfig.idleMode(IdleMode.kCoast);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollers.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    this.log("manipulator/angle", getEncoderPosition().in(Units.Rotations));
    this.log("manipulator/rollers appl out", rollers.getAppliedOutput());
    this.log("manipulator/setpoint", setpoint);
    this.log("manipulator/pivot appl out", pivot.getAppliedOutput());

    this.error = Math.abs(getEncoderPosition().in(Rotations) - setpoint);
    this.log("manipulator/error", error);
    this.log("manipulator/at setpoint", atSetpoint);

    double ff = Math.sin(getEncoderPosition().in(Units.Radians)) * kFF;
    this.log("manipulator/ff", ff);
    pivotController.setReference(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kPercentOut);

    pivotChecker.updateFaults();
    rollersChecker.updateFaults();
  }

  private double getAngle(GameState state) {
    return Constants.reefMap.get(state).angle().in(Units.Rotations);
  }

  public void setAngle(GameState state) {
    this.setpoint = getAngle(state);
    this.atSetpoint = false;
  }

  private Angle getEncoderPosition() {
    return Rotations.of(Math.abs(pivotEncoder.getPosition()));
  }

  /** Check if pivot is at angle setpoint to some degree of error */
  public boolean atAngle() {
    this.atSetpoint = error < ManipulatorConstants.kRotTolerance.in(Rotations);
    return atSetpoint;
  }

  /** Spin rollers forward or backward at default speed */
  public Command spinRollers(boolean forward) {
    int multipler = forward == true ? 1 : -1;
    return this.runOnce(() -> rollers.set(kDefaultRollerSpeed * multipler));
  }

  public Command stopRollers() {
    return this.runOnce(() -> rollers.set(0.0));
  }

  public void stop() {
    pivot.stopMotor();
    rollers.stopMotor();
  }

  @Override
  public void close() throws Exception {
    pivot.close();
    rollers.close();
  }
}
