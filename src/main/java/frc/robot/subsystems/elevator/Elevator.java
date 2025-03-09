package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants.Config;
import monologue.Logged;

public class Elevator extends SubsystemBase implements AutoCloseable, Logged {
  private SparkMax sparkMaxLeader;
  private SparkMax sparkMaxFollower;
  public SparkClosedLoopController pidControllerLeader;

  public Elevator(SparkMax sparkMaxLeader, SparkMax sparkMaxFollower) {
    this.sparkMaxLeader = sparkMaxLeader;
    this.sparkMaxFollower = sparkMaxFollower;
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    pidControllerLeader = sparkMaxLeader.getClosedLoopController();

    globalConfig
        .idleMode(Config.idleMode)
        .closedLoop
        .feedbackSensor(Config.feedbackSensor)
        .pidf(Config.pidP, Config.pidI, Config.pidD, Config.feedForward);
    leaderConfig.apply(globalConfig).inverted(Config.inverted);
    followerConfig.apply(globalConfig).follow(sparkMaxLeader);

    sparkMaxLeader.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkMaxFollower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double speed) {
    sparkMaxLeader.set(speed);
  }

  @Override
  public void close() {
    sparkMaxFollower.close();
    sparkMaxLeader.close();
  }
}
