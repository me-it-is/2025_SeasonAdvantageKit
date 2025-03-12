package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Config;
import frc.robot.util.SparkMaxFaultChecker;
import frc.robot.Constants.GameState;
import monologue.Logged;

public class Elevator extends SubsystemBase implements AutoCloseable, Logged {
  private SparkMax sparkMaxLeader;
  private SparkMax sparkMaxFollower;
  private SparkMaxFaultChecker leaderChecker;
  private SparkMaxFaultChecker followerChecker;
  private RelativeEncoder encoder;
  public SparkClosedLoopController pidControllerLeader;
  SparkMaxConfig globalConfig;
  SparkMaxConfig followerConfig;
  SparkMaxConfig leaderConfig;
  private Distance setpoint = Constants.reefMap.get(GameState.NONE).distance();

  public Elevator(SparkMax sparkMaxLeader, SparkMax sparkMaxFollower) {
    this.sparkMaxLeader = sparkMaxLeader;
    this.sparkMaxFollower = sparkMaxFollower;
    this.leaderChecker = new SparkMaxFaultChecker(sparkMaxLeader);
    this.followerChecker = new SparkMaxFaultChecker(sparkMaxFollower);
    this.encoder = sparkMaxLeader.getEncoder();
    this.globalConfig = new SparkMaxConfig();
    this.followerConfig = new SparkMaxConfig();
    this.leaderConfig = new SparkMaxConfig();
    this.pidControllerLeader = sparkMaxLeader.getClosedLoopController();

    encoder.setPosition(0);
    globalConfig
        .encoder
        .positionConversionFactor(Config.positionConversionFactor);
    globalConfig
        .closedLoop
        .pidf(Config.pidP, Config.pidI, Config.pidD, Config.feedForward);
    leaderConfig.apply(globalConfig).inverted(Config.inverted);
    followerConfig.apply(globalConfig).follow(sparkMaxLeader);

    sparkMaxLeader.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkMaxFollower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    this.log("elevator/height meters", getElevatorHeight().in(Meters));
    this.log("elevator/setpoint meters", setpoint.in(Units.Meters));
    this.log("elevator/leader output", sparkMaxLeader.get());
    this.log("elevator/follower output", sparkMaxFollower.get());
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();
    pidControllerLeader.setReference(
      (setpoint.in(Meters) / ElevatorConstants.maxHeight.in(Meters)), ControlType.kPosition);

    leaderChecker.checkFaults();
    followerChecker.checkFaults();
  }

  public boolean atSetpoint() {
    return Math.abs(setpoint.in(Meters) - getElevatorHeight().in(Meters)) < ElevatorConstants.setpointTolerance.in(Meters);
  }

  public Distance getElevatorHeight() {
    return Meters.of(encoder.getPosition());
  }

  @Override
  public void close() {
    sparkMaxFollower.close();
    sparkMaxLeader.close();
  }
}
