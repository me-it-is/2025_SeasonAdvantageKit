package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GameState;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.util.SparkMaxFaultChecker;
import monologue.Logged;

public class Manipulator extends SubsystemBase implements Logged, AutoCloseable {
  private SparkMaxConfig pivotConfig;
  private SparkMax pivot;
  private SparkMax rollers;
  private SparkAbsoluteEncoder pivotEncoder;
  private SparkClosedLoopController pivotController;
  private SparkMaxFaultChecker pivotChecker;
  private SparkMaxFaultChecker rollersChecker;
  private DigitalInput coralDetect;

  public Manipulator(SparkMax pivot, SparkMax rollers, DigitalInput linebreakSensor) {
    pivotConfig = new SparkMaxConfig();
    this.pivot = pivot;
    this.rollers = rollers;
    this.coralDetect = linebreakSensor;
    pivotEncoder = pivot.getAbsoluteEncoder();
    pivotController = pivot.getClosedLoopController();
    pivotChecker = new SparkMaxFaultChecker(pivot);
    rollersChecker = new SparkMaxFaultChecker(rollers);

    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    this.log("manipulator/angle", pivotEncoder.getPosition());
    this.log("manipulator/rollers", rollers.get());
    this.log("manipulator/hasCoral", hasCoral());

    pivotChecker.checkFaults();
    rollersChecker.checkFaults();
  }

  private double getAngle(GameState state) {
    return Constants.reefMap.get(state).angle().in(Units.Rotations);
  }

  public Command setAngle(GameState state) {
    double setpoint = getAngle(state);
    return this.run(
        () -> {
          double ff = Math.cos(getEncoderPosition().in(Units.Radians)) * kFF;
          pivotController.setReference(
              setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kPercentOut);
        });
  }

  private Angle getEncoderPosition() {
    return Rotations.of(pivotEncoder.getPosition());
  }

  public boolean atAngle(GameState state) {
    double setpoint = getAngle(state);
    return Math.abs(getEncoderPosition().in(Rotations) - setpoint)
        < ManipulatorConstants.rotTolerance.in(Rotations);
  }

  public Command spinRollers(boolean forward) {
    int multipler = forward == true ? 1 : -1;
    return this.runOnce(() -> rollers.set(defaultRollerSpeed * multipler));
  }

  public Command stopRollers() {
    return this.runOnce(() -> rollers.set(0.0));
  }

  public boolean hasCoral() {
    return coralDetect.get();
  }

  @Override
  public void close() throws Exception {
    pivot.close();
    rollers.close();
    coralDetect.close();
  }
}
