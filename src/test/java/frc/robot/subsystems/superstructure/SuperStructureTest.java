package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.Date;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SuperStructureTest {
  Manipulator manipulator;
  Elevator elevator;
  TalonFX leaderController;
  TalonFX followerController;
  TalonFXSimState leaderSim;
  TalonFXSimState followerSim;
  Distance setpoint = Constants.reefMap.get(GameState.NONE).distance();
  double DELTA = 1e-2;
  SparkMax pivot;
  SparkMax rollers;
  SparkMaxSim pivotSim;
  SparkMaxSim rollerSim;
  SparkAbsoluteEncoderSim manipEncoderSim;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 1);
    this.leaderController = new TalonFX(ElevatorConstants.kTalonLeaderCANId, "blinky");
    this.followerController = new TalonFX(ElevatorConstants.kTalonFollowerCANId, "blinky");

    this.pivot = new SparkMax(ManipulatorConstants.kPivotId, MotorType.kBrushless);
    this.rollers = new SparkMax(ManipulatorConstants.kRollerId, MotorType.kBrushless);

    this.manipulator = new Manipulator(pivot, rollers);
    this.elevator = new Elevator(leaderController, followerController);

    this.leaderSim = new TalonFXSimState(leaderController);
    this.followerSim = new TalonFXSimState(followerController);

    this.pivotSim = new SparkMaxSim(pivot, DCMotor.getNEO(1));
    this.rollerSim = new SparkMaxSim(rollers, DCMotor.getNEO(1));

    this.manipEncoderSim = pivotSim.getAbsoluteEncoderSim();
    CommandScheduler.getInstance().enable();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  void shutdown() throws Exception {
    elevator.close();
    manipulator.close();
  }

  @Test
  void testMoveToState() {
    boolean auto = false;
    for (GameState stage : GameState.values()) {
      Command cmd =
          RobotContainer.moveToState(stage, auto, elevator, manipulator).ignoringDisable(true);
      cmd.addRequirements(elevator, manipulator);

      CommandScheduler.getInstance().schedule(cmd);
      Date startTime = new Date();
      Date currentTime = startTime;
      while (CommandScheduler.getInstance().isScheduled(cmd)
          && (currentTime.getTime() - startTime.getTime()) < ((auto ? 2000 : 500) + 1000)) {
        CommandScheduler.getInstance().run();
        currentTime = new Date();
      }

      assertEquals(
          Constants.reefMap.get(stage).distance().in(Meters), elevator.setpoint.in(Meters), DELTA);
      assertEquals(
          Constants.reefMap.get(stage).angle().in(Rotations),
          manipulator.setpoint.in(Rotations),
          DELTA);

      leaderSim.setRawRotorPosition(
          elevator.heightToAngle(Constants.reefMap.get(stage).distance()));
      manipEncoderSim.setPosition(Constants.reefMap.get(stage).angle().in(Rotations));
      elevator.preformBlockingRefereshOnAllSignals();

      assertTrue(elevator.atSetpoint());
      assertTrue(manipulator.atAngle());

      CommandScheduler.getInstance().run();
      assertEquals(0, rollerSim.getSetpoint(), DELTA);
    }
  }
}
