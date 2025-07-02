package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;

import frc.robot.subsystems.manipulator.ManipulatorIO.ManipulatorIOInputs;

public class ManipulatorIOSparkMaxSim implements ManipulatorIO {
  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    inputs.manipulatorAngle = Rotations.zero();

    inputs.rollerOutput = 0;
    inputs.beamBroken = false;
    inputs.pivotOutput = 0;
  }
}
