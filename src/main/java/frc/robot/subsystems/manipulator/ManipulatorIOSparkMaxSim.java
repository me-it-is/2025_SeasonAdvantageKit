package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;

public class ManipulatorIOSparkMaxSim extends ManipulatorIOSparkMax {
  private DCMotor pivotGearbox = DCMotor.getNEO(1);
  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          pivotGearbox,
          16,
          SingleJointedArmSim.estimateMOI(0.207, 2.70547),
          0.207,
          -Math.PI / 2,
          Math.PI / 2,
          true,
          -Math.PI / 2);
  private DCMotor rollerGearbox = DCMotor.getNEO(1);
  private SparkMaxSim pivotMotorSim;
  private SparkMaxSim rollerMotorSim;

  public ManipulatorIOSparkMaxSim(SparkMax pivot, SparkMax rollers) {
    super(pivot, rollers);

    rollerMotorSim = new SparkMaxSim(rollers, rollerGearbox);
    pivotMotorSim = new SparkMaxSim(pivot, pivotGearbox);
    pivotMotorSim.getAbsoluteEncoderSim().setPosition(-0.25);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    Logger.recordOutput(
        "ManipulatorSim/voltsOut",
        pivotMotorSim.getAppliedOutput() * pivotMotorSim.getBusVoltage());
    pivotSim.setInputVoltage(pivotMotorSim.getAppliedOutput() * pivotMotorSim.getBusVoltage());

    pivotSim.update(Constants.kDt);
    Logger.recordOutput("ManipulatorSim/rads", pivotSim.getAngleRads());
    Logger.recordOutput("ManipulatorSim/radsPerSecond", pivotSim.getVelocityRadPerSec());

    pivotMotorSim.iterate(
        RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()).in(RotationsPerSecond) / 60,
        SimulatedBattery.getBatteryVoltage().in(Volts),
        Constants.kDt);
    pivotMotorSim.setPosition(Radians.of(pivotSim.getAngleRads()).in(Rotations));
    super.updateInputs(inputs);

    inputs.rollerOutput = rollerMotorSim.getAppliedOutput();
    inputs.beamBroken = false;
    inputs.pivotOutput = pivotMotorSim.getAppliedOutput();
  }
}
