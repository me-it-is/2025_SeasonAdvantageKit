package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;

public class ClimberIOSparkSim implements ClimberIO {
  public ClimberIOSparkSim() {}

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    inputs.climberAngle = Rotations.zero();
    inputs.motorOutput = 0;
  }
}
