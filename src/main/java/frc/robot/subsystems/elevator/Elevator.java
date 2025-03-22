package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import frc.robot.util.RobotMath;
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
  private State currentState;
  private State profileSetpoint;
  private static double kDt = 0.02;
  private Distance setpointError = Meters.zero();
  private boolean atSetpoint = false;

  private boolean usingMotionProfile = false;

  public Elevator(SparkMax sparkMaxLeader, SparkMax sparkMaxFollower) {
    this.sparkMaxLeader = sparkMaxLeader;
    this.sparkMaxFollower = sparkMaxFollower;
    this.leaderChecker = new SparkFaultChecker(sparkMaxLeader, "elevator leader");
    this.followerChecker = new SparkFaultChecker(sparkMaxFollower, "elevator follower");
    this.encoder = sparkMaxLeader.getEncoder();
    this.pidControllerLeader = sparkMaxLeader.getClosedLoopController();

    this.profile = ElevatorConstants.getProfile();
    this.m_goal = new TrapezoidProfile.State();
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
    currentState = profileSetpoint;
    profileSetpoint = profile.calculate(kDt, profileSetpoint, m_goal);

    this.setpointError = RobotMath.abs(getElevatorHeight().minus(setpoint));
    this.atSetpoint = setpointError.lt(ElevatorConstants.kSetpointTolerance);
    if (usingMotionProfile) {
      pidControllerLeader.setReference(
          profileSetpoint.velocity,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          ElevatorConstants.kFFCalculator.calculateWithVelocities(
              currentState.velocity, profileSetpoint.velocity),
          ArbFFUnits.kVoltage);
    }

    this.log("elevator/error", setpointError.in(Meters));
    this.log("elevator/at setpoint", atSetpoint);
    this.log("elevator/height meters", getElevatorHeight().in(Meters));
    this.log("elevator/setpoint meters", setpoint.in(Units.Meters));
    this.log("elevator/leader appl out", sparkMaxLeader.getAppliedOutput());
    this.log("elevator/follower appl out", sparkMaxFollower.getAppliedOutput());
    if (usingMotionProfile) {
      this.log("elevator/profile setpoint pos", profileSetpoint.position);
      this.log("elevator/profile setpoint vel", profileSetpoint.velocity);
    }
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();

    this.m_goal = new TrapezoidProfile.State(heightToAngle(setpoint).in(Rotations), 0);
    if (!usingMotionProfile) {
      pidControllerLeader.setReference(
          heightToAngle(setpoint).in(Rotations), ControlType.kPosition);
    }

    leaderChecker.updateFaults();
    followerChecker.updateFaults();
  }

  public boolean atSetpoint() {
    return atSetpoint;
  }

  public Distance getElevatorHeight() {
    return angleToHeight(Rotations.of(encoder.getPosition()));
  }

  private Distance angleToHeight(Angle angle) {
    return (Distance) ElevatorConstants.kAngularSpan.timesDivisor(angle);
  }

  private Angle heightToAngle(Distance height) {
    return (Angle) ElevatorConstants.kSpanAngle.timesDivisor(height);
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
