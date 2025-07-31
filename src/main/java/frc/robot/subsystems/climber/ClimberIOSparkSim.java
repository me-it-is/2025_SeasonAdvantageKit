package frc.robot.subsystems.climber;

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

public class ClimberIOSparkSim extends ClimberIOSpark {
  private DCMotor gearbox = DCMotor.getNEO(1);
  private SingleJointedArmSim physicsSim =
      new SingleJointedArmSim(
          gearbox,
          25,
          SingleJointedArmSim.estimateMOI(0.270239, 1.31138),
          0.270239,
          0,
          Math.PI,
          false,
          0);
  private SparkMaxSim motorSim;

  public ClimberIOSparkSim(SparkMax motorController) {
    super(motorController);

    motorSim = new SparkMaxSim(motorController, gearbox);
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    physicsSim.setInputVoltage(motorSim.getAppliedOutput() * motorSim.getBusVoltage());
    physicsSim.update(Constants.kDt);

    motorSim.iterate(
        RadiansPerSecond.of(physicsSim.getVelocityRadPerSec()).in(RotationsPerSecond),
        SimulatedBattery.getBatteryVoltage().in(Volts),
        Constants.kDt);
    motorSim.setPosition(Radians.of(physicsSim.getAngleRads()).in(Rotations));

    inputs.climberAngle = Radians.of(physicsSim.getAngleRads());
    inputs.motorOutput = motorSim.getAppliedOutput();
  }
}
