package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;

public class ElevatorIOTalonFXSim implements ElevatorIO {
  public ElevatorIOTalonFXSim() {}

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    inputs.leaderPosition = Rotations.zero();
    inputs.leaderVelocity = RotationsPerSecond.zero();
    inputs.leaderVoltage = Volts.zero();
    inputs.leaderCurrent = Amps.zero();
    inputs.leaderStatus = StatusCode.OK;

    inputs.followerPosition = Rotations.zero();
    inputs.followerVelocity = RotationsPerSecond.zero();
    inputs.followerVoltage = Volts.zero();
    inputs.followerCurrent = Amps.zero();
    inputs.followerStatus = StatusCode.OK;
  }
}
