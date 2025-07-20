package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.Elevator.heightToAngle;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import java.util.function.Supplier;

public class ElevatorIOTalonFXSim extends ElevatorIOTalonFX {
  TalonFXSimState leaderSim;
  Supplier<Voltage> batteryVoltage;

  private static final DCMotor kSimGearBox = DCMotor.getKrakenX60(2);
  private static final ElevatorSim elevatorSim =
      new ElevatorSim(
          kSimGearBox,
          kGearRatio,
          kElevatorMass.in(Kilograms),
          Millimeters.of(18).in(Meters),
          0,
          kMaxHeight.in(Meters),
          true,
          0,
          0.1,
          0);

  public ElevatorIOTalonFXSim(
      TalonFX talonLeader, TalonFX talonFollower, Supplier<Voltage> batteryVoltage) {
    super(talonLeader, talonFollower);

    leaderSim = talonLeader.getSimState();
    this.batteryVoltage = batteryVoltage;
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    leaderSim.setSupplyVoltage(batteryVoltage.get());

    elevatorSim.setInputVoltage(leaderSim.getMotorVoltage());
    elevatorSim.update(Constants.kDt);

    leaderSim.setRawRotorPosition(
        heightToAngle(Meters.of(elevatorSim.getPositionMeters())).times(kGearRatio));
    leaderSim.setRotorVelocity(
        RadiansPerSecond.of(
            heightToAngle(Meters.of(elevatorSim.getVelocityMetersPerSecond()))
                .times(kGearRatio)
                .in(Radians)));

    super.updateInputs(inputs);

    inputs.followerPosition = inputs.leaderPosition;
    inputs.followerVelocity = inputs.leaderVelocity;
    inputs.followerVoltage = inputs.leaderVoltage;
    inputs.followerCurrent = inputs.leaderCurrent;
    inputs.followerStatus = inputs.leaderStatus;
  }
}
