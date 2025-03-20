package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import frc.robot.util.faultChecker.SparkFaultChecker;
import monologue.Logged;

public class Elevator extends SubsystemBase implements AutoCloseable, Logged {
  private SparkMax sparkMaxLeader;
  private SparkMax sparkMaxFollower;
  private SparkFaultChecker leaderChecker;
  private SparkFaultChecker followerChecker;
  private RelativeEncoder encoder;
  public SparkClosedLoopController pidControllerLeader;
  private Distance setpoint = Constants.reefMap.get(GameState.NONE).distance();
  private TrapezoidProfile profile;
  private State m_goal;
  private State m_prevGoal;
  private State profileSetpoint;
  private static double kDt = 0.02;
  

  public Elevator(SparkMax sparkMaxLeader, SparkMax sparkMaxFollower) {
    this.sparkMaxLeader = sparkMaxLeader;
    this.sparkMaxFollower = sparkMaxFollower;
    this.leaderChecker = new SparkFaultChecker(sparkMaxLeader, "elevator leader");
    this.followerChecker = new SparkFaultChecker(sparkMaxFollower, "elevator follower");
    this.encoder = sparkMaxLeader.getEncoder();
    this.pidControllerLeader = sparkMaxLeader.getClosedLoopController();
    this.profile = ElevatorConstants.getProfile();
    this.m_goal = new TrapezoidProfile.State();
    this.m_prevGoal = new TrapezoidProfile.State();
    this.profileSetpoint = new TrapezoidProfile.State();

    encoder.setPosition(0);
    sparkMaxLeader.configure(
        ElevatorConstants.getLeaderConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    sparkMaxFollower.configure(
        ElevatorConstants.getFollowerConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    this.log("elevator/height meters", getElevatorHeight().in(Meters));
    this.log("elevator/setpoint meters", setpoint.in(Units.Meters));
    this.log("elevator/leader output", sparkMaxLeader.get());
    this.log("elevator/follower output", sparkMaxFollower.get());

    if (!m_prevGoal.equals(m_goal)) {
      System.out.println("reinitializing profile");
      profile = ElevatorConstants.getProfile(); // reinitialize the profile to calculate new motion trajectory
      m_prevGoal = new State(m_goal.position, m_goal.velocity);
    }

    profileSetpoint = profile.calculate(kDt, profileSetpoint, m_goal);
    this.log("elevator/profile setpoint pos", profileSetpoint.position);
    this.log("elevator/profile setpoint vel", profileSetpoint.velocity);
    pidControllerLeader.setReference(
        profileSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ElevatorConstants.kFF * profileSetpoint.velocity,
        ArbFFUnits.kPercentOut);
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();
    this.m_goal = new TrapezoidProfile.State(metersToRots(setpoint.in(Units.Meters)), 0);

    leaderChecker.updateFaults();
    followerChecker.updateFaults();
  }

  public boolean atSetpoint() {
    return Math.abs(setpoint.in(Meters) - getElevatorHeight().in(Meters))
        < ElevatorConstants.kSetpointTolerance.in(Meters);
  }

  public Distance getElevatorHeight() {
    return Meters.of(rotsToMeters(encoder.getPosition()));
  }

  private double rotsToMeters(double rots) {
    return rots / ElevatorConstants.kRotsPerFullExtension * ElevatorConstants.kMaxHeight.in(Meters);
  }

  private double metersToRots(double meters) {
    return meters
        / ElevatorConstants.kMaxHeight.in(Meters)
        * ElevatorConstants.kRotsPerFullExtension;
  }

  public void stop() {
    sparkMaxLeader.stopMotor();
  }

  @Override
  public void close() {
    sparkMaxFollower.close();
    sparkMaxLeader.close();
  }
}
