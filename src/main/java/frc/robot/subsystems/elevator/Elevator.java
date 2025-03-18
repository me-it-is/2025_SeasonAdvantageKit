package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import monologue.Logged;

public class Elevator extends SubsystemBase implements AutoCloseable, Logged {
  private TalonFX talonLeader;
  private Follower talonFollower;
  private Distance setpoint;
  private TrapezoidProfile profile;
  private State goal;
  private State profileSetpoint;
  private static double kDt = 0.02;

  public Elevator(TalonFX talonLeader, Follower talonFollower) {
    this.talonLeader = talonLeader;
    this.talonFollower = talonFollower;
    this.setpoint = Constants.reefMap.get(GameState.NONE).distance();
    this.profile =
        new TrapezoidProfile(
            new Constraints(
                ElevatorConstants.kMaxVelocity.in(Units.RotationsPerSecond),
                ElevatorConstants.kMaxAcceleration.in(Units.RotationsPerSecondPerSecond)));
    this.goal = new TrapezoidProfile.State();
    this.profileSetpoint = new TrapezoidProfile.State();

    talonLeader.getConfigurator().apply(ElevatorConstants.getSharedConfig());
    talonFollower.OpposeMasterDirection = true;
  }

  @Override
  public void periodic() {
    this.log("elevator/height meters", getElevatorHeight().in(Meters));
    this.log("elevator/setpoint meters", setpoint.in(Meters));
    this.log("elevator/leader output", talonLeader.get());

    this.profileSetpoint = profile.calculate(kDt, profileSetpoint, goal);
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();
    double rotationSetpoint =
        setpoint.in(Meters)
            / ElevatorConstants.kMaxHeight.in(Meters)
            * ElevatorConstants.kRotsPerFullExtension;

    this.goal = new TrapezoidProfile.State(rotationSetpoint, 0);
    this.profileSetpoint = new TrapezoidProfile.State();
    PositionVoltage motorRequest = new PositionVoltage(0).withSlot(0);
    this.profileSetpoint =
        this.profile.calculate(
            ElevatorConstants.totalExtensionTime.in(Seconds), profileSetpoint, goal);

    motorRequest.Position = profileSetpoint.position;
    motorRequest.Velocity = profileSetpoint.velocity;
    motorRequest.EnableFOC = true;
    talonLeader.setControl(motorRequest);
  }

  public boolean atSetpoint() {
    return talonLeader.getClosedLoopError().getValue()
        < ElevatorConstants.kSetpointTolerance.in(Meters)
            * ElevatorConstants.kPositionConversionFactor;
  }

  public Distance getElevatorHeight() {
    return Meters.of(
        talonLeader.getPosition().getValue().in(Rotations)
            * ElevatorConstants.kPositionConversionFactor);
  }

  public void stop() {
    talonLeader.stopMotor();
  }

  @Override
  public void close() {
    talonLeader.close();
  }
}
