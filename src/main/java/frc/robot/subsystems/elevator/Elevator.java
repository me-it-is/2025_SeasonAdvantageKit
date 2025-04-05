package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.kFF;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
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
  private State goalState;
  private State currentState;
  private State nextState;
  private static double kDt = 0.02;
  private Distance setpointError;
  private boolean atSetpoint;

  private boolean usingMotionProfile = true;
  private boolean usingVoltageControl = false;

  public Elevator(SparkMax sparkMaxLeader, SparkMax sparkMaxFollower) {
    this.sparkMaxLeader = sparkMaxLeader;
    this.sparkMaxFollower = sparkMaxFollower;
    this.leaderChecker = new SparkFaultChecker(sparkMaxLeader, "elevator leader");
    this.followerChecker = new SparkFaultChecker(sparkMaxFollower, "elevator follower");
    this.encoder = sparkMaxLeader.getEncoder();
    this.pidControllerLeader = sparkMaxLeader.getClosedLoopController();

    this.profile = ElevatorConstants.getProfile();
    this.goalState = new TrapezoidProfile.State();
    this.nextState = new TrapezoidProfile.State();

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
    currentState = nextState;
    nextState = profile.calculate(kDt, currentState, goalState);

    this.setpointError = RobotMath.abs(getElevatorHeight().minus(setpoint));
    this.atSetpoint = setpointError.lt(ElevatorConstants.kSetpointTolerance);

    if (encoder.getPosition() > 20 || encoder.getPosition() < -0.1) {
      close();
    }

    if (usingMotionProfile && !usingVoltageControl) {
      pidControllerLeader.setReference(
          nextState.velocity,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          ElevatorConstants.kFFCalculator.calculateWithVelocities(
              currentState.velocity, nextState.velocity),
          ArbFFUnits.kVoltage);
    }

    this.log("elevator/error", setpointError.in(Meters));
    this.log("elevator/at setpoint", atSetpoint);
    this.log("elevator/height rotations", heightToAngle(getElevatorHeight()).in(Rotations));
    this.log("elevator/setpoint meters", setpoint.in(Units.Meters));
    this.log("elevator/leader appl out", sparkMaxLeader.getAppliedOutput());
    this.log("elevator/follower appl out", sparkMaxFollower.getAppliedOutput());
    if (usingMotionProfile) {
      this.log("elevator/profile setpoint pos", nextState.position);
      this.log("elevator/profile setpoint vel", nextState.velocity);
    }

    leaderChecker.updateFaults();
    followerChecker.updateFaults();
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();

    this.goalState = new TrapezoidProfile.State(heightToAngle(setpoint).in(Rotations), 0);
    if (!usingMotionProfile && !usingVoltageControl) {
      pidControllerLeader.setReference(
          heightToAngle(setpoint).in(Rotations),
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          kFF,
          ArbFFUnits.kVoltage);
    }
  }

  public boolean atSetpoint() {
    return atSetpoint;
  }

  public void voltageDrive(Voltage volts) {
    this.usingVoltageControl = true;
    this.log("elevator/applied voltage in routine", -volts.in(Volts));
    sparkMaxLeader.setVoltage(-volts.in(Volts));
  }

  public void sysIdLog(SysIdRoutineLog log) {
    log.motor("Elevator motors")
        .voltage(Volts.of(sparkMaxLeader.getBusVoltage() * sparkMaxLeader.getAppliedOutput()))
        .linearPosition(getElevatorHeight())
        .linearVelocity(getElevatorVelocity());
  }

  public Distance getElevatorHeight() {
    return angleToHeight(Rotations.of(encoder.getPosition()));
  }

  public LinearVelocity getElevatorVelocity() {
    return RobotMath.castToMoreSpecificUnits(
        RobotMath.useConversionFactorFromLowerOrderUnitForHigherOrderConversion(
            ElevatorConstants.kAngularSpan, RotationsPerSecond.of(encoder.getVelocity())),
        MetersPerSecond.zero());
  }

  private Distance angleToHeight(Angle angle) {
    return (Distance) ElevatorConstants.kAngularSpan.timesDivisor(angle);
  }

  private Angle heightToAngle(Distance height) {
    return (Angle) ElevatorConstants.kSpanAngle.timesDivisor(height);
  }

  public void hold() {
    pidControllerLeader.setReference(
        encoder.getPosition(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot1,
        kFF,
        ArbFFUnits.kVoltage);
  }

  public void setUseVoltageControl(boolean useVoltage) {
    this.usingVoltageControl = useVoltage;
  }

  public void zeroElevator() {
    this.nextState = new State(0, 0);
    this.goalState = new State(0, 0);
    encoder.setPosition(0);
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
