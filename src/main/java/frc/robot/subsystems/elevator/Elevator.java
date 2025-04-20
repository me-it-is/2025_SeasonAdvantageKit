package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.kFF;
import static frc.robot.Constants.ElevatorConstants.kGearRatio;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double kP = ElevatorConstants.kP;
  private double kI = ElevatorConstants.kI;
  private double kD = ElevatorConstants.kD;

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

    SmartDashboard.putNumber("elevator/kP", kP);
    SmartDashboard.putNumber("elevator/kI", kI);
    SmartDashboard.putNumber("elevator/kD", kD);
    SmartDashboard.putNumber("elevator/kFF", kFF);
  }

  public boolean isWithinUpperBounds() {
    return getElevatorHeight().lt(ElevatorConstants.kMaxHeight.times(0.75));
  }

  public boolean isWithinLowerBounds() {
    return getElevatorHeight().gt(ElevatorConstants.kMaxHeight.times(0.25));
  }

  @Override
  public void periodic() {
    currentState = new State(encoder.getPosition(), encoder.getVelocity());
    nextState = profile.calculate(kDt, currentState, goalState);
    this.log("elevator/goal state position", goalState.position);
    this.log("elevator/goal state velocity", goalState.velocity);

    this.setpointError = getElevatorHeight().minus(setpoint);
    this.atSetpoint = setpointError.lt(ElevatorConstants.kSetpointTolerance);

    double dashkP = SmartDashboard.getNumber("elevator/kP", kP);
    double dashkI = SmartDashboard.getNumber("elevator/kI", kI);
    double dashkD = SmartDashboard.getNumber("elevator/kD", kD);
    if (kP != dashkP || kI != dashkI || kD != dashkD) {
      SparkMaxConfig newConfig = new SparkMaxConfig();
      newConfig.closedLoop.pid(dashkP, dashkI, dashkD).outputRange(-0.5, 0.5);
      sparkMaxLeader.configure(
          ElevatorConstants.getLeaderConfig().apply(newConfig),
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
      sparkMaxFollower.configure(
          ElevatorConstants.getFollowerConfig().apply(newConfig),
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
      kP = dashkP;
      kI = dashkI;
      kD = dashkD;
    }

    if (usingMotionProfile && !usingVoltageControl) {
      LinearVelocity curLinearVel =
          getElevatorVelocity(RotationsPerSecond.of(currentState.velocity));
      LinearVelocity nextLinearVel = getElevatorVelocity(RotationsPerSecond.of(nextState.velocity));

      double ffOut =
          ElevatorConstants.kFFCalculator.calculateWithVelocities(
              curLinearVel.in(MetersPerSecond), nextLinearVel.in(MetersPerSecond));
      this.log("elevator/ffOut", ffOut);

      pidControllerLeader.setReference(
          nextState.velocity
              / kGearRatio, // encoder is on motor shaft, so spins kGearRatio times per physical
          // rotation
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          ffOut,
          ArbFFUnits.kVoltage);
    }

    this.log("elevator/error rots", heightToAngle(setpointError).in(Rotations));
    this.log("elevator/at setpoint", atSetpoint);
    this.log("elevator/height rotations", encoder.getPosition());
    this.log("elevator/height meters", getElevatorHeight().in(Meters));
    this.log("elevator/setpoint meters", setpoint.in(Units.Meters));
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
    sparkMaxLeader.setVoltage(volts.in(Volts));
  }

  public void sysIdLog(SysIdRoutineLog log) {
    log.motor("Elevator motors")
        .voltage(Volts.of(sparkMaxLeader.getBusVoltage() * sparkMaxLeader.getAppliedOutput()))
        .linearPosition(getElevatorHeight())
        .linearVelocity(getElevatorVelocity(RotationsPerSecond.of(encoder.getVelocity())));
  }

  public Distance getElevatorHeight() {
    return angleToHeight(Rotations.of(encoder.getPosition()));
  }

  private LinearVelocity getElevatorVelocity(AngularVelocity velocity) {
    return RobotMath.castToMoreSpecificUnits(
        RobotMath.useConversionFactorFromLowerOrderUnitForHigherOrderConversion(
            ElevatorConstants.kAngularSpan, velocity),
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
