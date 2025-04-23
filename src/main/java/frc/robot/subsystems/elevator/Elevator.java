package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import frc.robot.util.RobotMath;
import frc.robot.util.faultChecker.CTREFaultChecker;

public class Elevator extends SubsystemBase implements AutoCloseable {
  private TalonFX talonLeader;
  private TalonFX talonFollower;
  private Distance setpoint = Constants.reefMap.get(GameState.NONE).distance();
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage motionVoltageRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;

  private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

  private CTREFaultChecker leadChecker;
  private CTREFaultChecker followerChecker;

  public Elevator(TalonFX talonLeader, TalonFX talonFollower) {
    this.talonLeader = talonLeader;
    this.talonFollower = talonFollower;
    talonFollower.setControl(new Follower(ElevatorConstants.kTalonLeaderCANId, true));

    var elevatorConfig = ElevatorConstants.elevatorConfig;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.Slot0 = ElevatorConstants.elevatorGains;

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 400;
    elevatorConfig.MotionMagic.MotionMagicJerk = 0;

    tryUntilOk(5, () -> talonLeader.getConfigurator().apply(elevatorConfig, 0.25));

    leaderPosition = talonLeader.getPosition();
    leaderVelocity = talonLeader.getVelocity();
    leaderAppliedVolts = talonLeader.getMotorVoltage();
    leaderCurrent = talonLeader.getStatorCurrent();

    followerPosition = talonFollower.getPosition();
    followerVelocity = talonFollower.getVelocity();
    followerAppliedVolts = talonFollower.getMotorVoltage();
    followerCurrent = talonFollower.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(talonLeader, talonFollower);

    this.leadChecker = new CTREFaultChecker(talonLeader, "Elevator leader");
    this.followerChecker = new CTREFaultChecker(talonFollower, "Elevator follower");
  }

  @Override
  public void periodic() {
    DogLog.log("elevator/height (meters)", getElevatorHeight().in(Meters));
    DogLog.log("elevator/setpoint (meters)", setpoint.in(Meters));
    DogLog.log("elevator/leader output", talonLeader.get());
    DogLog.log("elevator/follower output", talonFollower.get());

    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    var followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition, followerVelocity, followerAppliedVolts, followerCurrent);

    DogLog.log("elevator/leader connected", leaderConnectedDebounce.calculate(leaderStatus.isOK()));
    DogLog.log(
        "elevator/follower connected", followerConnectedDebounce.calculate(followerStatus.isOK()));
    DogLog.log("elevator/leader position (rots)", leaderPosition.getValueAsDouble());
    DogLog.log("elevator/follower position (rots)", followerPosition.getValueAsDouble());
    DogLog.log("elevator/leader velocity (rots / sec)", leaderVelocity.getValueAsDouble());
    DogLog.log("elevator/follower velocity (rots / sec)", followerVelocity.getValueAsDouble());
    DogLog.log("elevator/leader voltage", leaderAppliedVolts.getValueAsDouble());
    DogLog.log("elevator/follower voltage", followerAppliedVolts.getValueAsDouble());
    DogLog.log("elevator/leader current (A)", leaderCurrent.getValueAsDouble());
    DogLog.log("elevator/follower current (A)", followerCurrent.getValueAsDouble());

    this.leadChecker.updateFaults();
    this.followerChecker.updateFaults();
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();
    double rotationSetpoint = heightToAngle(setpoint).in(Rotations);
    talonLeader.setControl(motionVoltageRequest.withPosition(rotationSetpoint));
  }

  public boolean atSetpoint() {
    return Math.abs(leaderPosition.getValueAsDouble() - heightToAngle(setpoint).in(Rotations))
        < ElevatorConstants.kSetpointTolerance.in(Rotations);
  }

  public void setVoltage(double volts) {
    talonLeader.setControl(voltageRequest.withOutput(volts));
  }

  public void sysIdLog(SysIdRoutineLog log) {
    log.motor("Elevator motors")
        .voltage(talonLeader.getMotorVoltage().getValue())
        .linearPosition(getElevatorHeight())
        .linearVelocity(
            getElevatorVelocity(
                Rotations.per(Minute).of(talonLeader.getVelocity().getValueAsDouble())));
  }

  public void voltageDrive(Voltage volts) {
    talonLeader.setControl(new VoltageOut(volts));
  }

  public Distance getElevatorHeight() {
    return angleToHeight(Rotations.of(talonLeader.getPosition().getValueAsDouble()));
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

  public void stop() {
    talonLeader.stopMotor();
  }

  @Override
  public void close() {
    talonLeader.close();
  }
}
