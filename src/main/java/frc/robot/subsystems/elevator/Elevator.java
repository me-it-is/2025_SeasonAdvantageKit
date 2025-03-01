package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Config;
import frc.robot.Constants.GameState;
import monologue.Logged;

public class Elevator extends SubsystemBase implements AutoCloseable, Logged {
  private SparkMax sparkMaxLeader =
      new SparkMax(ElevatorConstants.sparkMaxCANId, MotorType.kBrushless);
  private SparkMax sparkMaxFollower =
      new SparkMax(ElevatorConstants.sparkMaxFollowerCANId, MotorType.kBrushless);
  private SparkAbsoluteEncoder encoder = sparkMaxLeader.getAbsoluteEncoder();
  public SparkClosedLoopController pidControllerLeader;

  private Distance setpoint = Meters.of(0);

  public Elevator() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    pidControllerLeader = sparkMaxLeader.getClosedLoopController();

    globalConfig
        .idleMode(Config.idleMode)
        .encoder
        .positionConversionFactor(Config.positionConvertionFactor);
    globalConfig
        .closedLoop
        .feedbackSensor(Config.feedbackSensor)
        .pidf(Config.pidP, Config.pidI, Config.pidD, Config.feedForward);
    leaderConfig.apply(globalConfig).inverted(Config.inverted);
    followerConfig.apply(globalConfig).follow(sparkMaxLeader);
    encoderConfig
        .positionConversionFactor(Config.positionConvertionFactor)
        .setSparkMaxDataPortConfig();

    sparkMaxLeader.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkMaxFollower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();
    setReference();
  }

  public void move(double speed) {
    sparkMaxLeader.set(speed);
  }

  private void setReference() {
    pidControllerLeader.setReference(
        (setpoint.in(Meters) / ElevatorConstants.maxHeight.in(Meters)), ControlType.kPosition);
  }

  public boolean atSetpoint() {
    return false;
  }

  @Override
  public void periodic() {
    this.log("Elevator height meters", getElevatorHeight().in(Meters));
  }

  public Distance getElevatorHeight() {
    return Meters.of(encoder.getPosition()).minus(ElevatorConstants.encoderOffset);
  }

  @Override
  public void close() {
    sparkMaxFollower.close();
    sparkMaxLeader.close();
  }
}
