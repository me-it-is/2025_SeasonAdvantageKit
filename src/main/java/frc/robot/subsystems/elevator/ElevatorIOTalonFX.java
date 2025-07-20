package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.Elevator.heightToAngle;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX talonLeader;
  private TalonFX talonFollower;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;

  private MotionMagicExpoVoltage profile =
      new MotionMagicExpoVoltage(Rotations.of(0)).withEnableFOC(true);

  public ElevatorIOTalonFX(TalonFX talonLeader, TalonFX talonFollower) {
    this.talonLeader = talonLeader;
    this.talonFollower = talonFollower;
    talonFollower.setControl(new Follower(ElevatorConstants.kTalonLeaderCANId, true));

    var elevatorConfig = ElevatorConstants.elevatorConfig;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.Slot0 = ElevatorConstants.elevatorGains0;
    elevatorConfig.Slot1 = ElevatorConstants.elevatorGains1;
    elevatorConfig.Slot2 = ElevatorConstants.elevatorGains2;

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 12;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 200;
    elevatorConfig.MotionMagic.MotionMagicJerk = 400;

    tryUntilOk(10, () -> talonLeader.getConfigurator().apply(elevatorConfig, 0.25));

    leaderPosition = talonLeader.getPosition();
    leaderVelocity = talonLeader.getVelocity();
    leaderAppliedVolts = talonLeader.getMotorVoltage();
    leaderCurrent = talonLeader.getStatorCurrent();

    followerPosition = talonFollower.getPosition();
    followerVelocity = talonFollower.getVelocity();
    followerAppliedVolts = talonFollower.getMotorVoltage();
    followerCurrent = talonFollower.getStatorCurrent();

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
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    inputs.leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition, followerVelocity, followerAppliedVolts, followerCurrent);

    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderVoltage = leaderAppliedVolts.getValue();
    inputs.leaderCurrent = leaderCurrent.getValue();

    inputs.followerPosition = followerPosition.getValue();
    inputs.followerVelocity = followerVelocity.getValue();
    inputs.followerVoltage = followerAppliedVolts.getValue();
    inputs.followerCurrent = followerCurrent.getValue();
  }

  @Override
  public void setSetpoint(Distance height) {
    talonLeader.setControl(profile.withPosition(heightToAngle(height)));
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
