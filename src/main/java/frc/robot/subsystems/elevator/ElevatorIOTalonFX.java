package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.Elevator.heightToAngle;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX talonLeader;
  private TalonFX talonFollower;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;

  private MotionMagicVoltage profile = new MotionMagicVoltage(Rotations.of(0)).withEnableFOC(true);

  public ElevatorIOTalonFX(TalonFX talonLeader, TalonFX talonFollower) {
    this.talonLeader = talonLeader;
    this.talonFollower = talonFollower;
    talonFollower.setControl(new Follower(ElevatorConstants.kTalonLeaderCANId, true));

    var elevatorConfig = ElevatorConstants.elevatorConfig;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    switch (Constants.currentMode) {
      default:
        elevatorConfig.Slot0 = ElevatorConstants.elevatorGains0;
      case SIM:
        elevatorConfig.Slot0 = ElevatorConstants.elevatorSimGains0;
    }

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = kMaxVelocity.in(RotationsPerSecond);
    elevatorConfig.MotionMagic.MotionMagicAcceleration =
        kMaxAcceleration.in(RotationsPerSecondPerSecond);
    elevatorConfig.MotionMagic.MotionMagicJerk =
        kMaxJerk.in(RotationsPerSecondPerSecond.per(Second));

    tryUntilOk(10, () -> talonLeader.getConfigurator().apply(elevatorConfig, 0.25));

    leaderPosition = talonLeader.getPosition();
    leaderVelocity = talonLeader.getVelocity();
    leaderAppliedVolts = talonLeader.getMotorVoltage();
    leaderCurrent = talonLeader.getTorqueCurrent();
    leaderSupplyCurrent = talonLeader.getSupplyCurrent();

    followerPosition = talonFollower.getPosition();
    followerVelocity = talonFollower.getVelocity();
    followerAppliedVolts = talonFollower.getMotorVoltage();
    followerCurrent = talonFollower.getTorqueCurrent();
    followerSupplyCurrent = talonFollower.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / Constants.kDt,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        leaderSupplyCurrent,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent,
        followerSupplyCurrent);

    ParentDevice.optimizeBusUtilizationForAll(talonLeader, talonFollower);
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    inputs.leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, leaderSupplyCurrent);
    inputs.followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition,
            followerVelocity,
            followerAppliedVolts,
            followerCurrent,
            followerSupplyCurrent);

    Logger.recordOutput(
        "Elevator/ClosedLoopReference", talonLeader.getClosedLoopReference().getValue());
    Logger.recordOutput("Elevator/ClosedLoopOutput", talonLeader.getClosedLoopOutput().getValue());
    Logger.recordOutput("Elevator/ClosedLoopFF", talonLeader.getClosedLoopFeedForward().getValue());

    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderVoltage = leaderAppliedVolts.getValue();
    inputs.leaderCurrent = leaderCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();

    inputs.followerPosition = followerPosition.getValue();
    inputs.followerVelocity = followerVelocity.getValue();
    inputs.followerVoltage = followerAppliedVolts.getValue();
    inputs.followerCurrent = followerCurrent.getValue();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();
  }

  @Override
  public void setSetpoint(Distance height) {
    talonLeader.setControl(profile.withPosition(heightToAngle(height)));
  }

  @Override
  public void voltageDrive(Voltage voltage) {
    talonLeader.setControl(new VoltageOut(voltage));
  }

  @Override
  public void stop() {
    talonLeader.stopMotor();
  }

  @Override
  public void Close() {
    talonLeader.close();
    talonFollower.close();
  }
}
