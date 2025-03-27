package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ElevatorTest {
  Elevator elevator;
  SparkMax leaderController;
  SparkMax followerController;
  SparkMaxSim leaderSim;
  SparkMaxSim followerSim;
  RelativeEncoder encoder;
  Distance setpoint = Constants.reefMap.get(GameState.NONE).distance();
  SparkRelativeEncoderSim encoderSim;
  double DELTA = 1e-2;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 1);
    this.leaderController = new SparkMax(ElevatorConstants.kSparkMaxCANId, MotorType.kBrushless);
    this.followerController =
        new SparkMax(ElevatorConstants.kSparkMaxFollowerCANId, MotorType.kBrushless);
    this.encoder = leaderController.getEncoder();
    this.elevator = new Elevator(leaderController, followerController);
    this.leaderSim = new SparkMaxSim(leaderController, DCMotor.getNEO(1));
    this.followerSim = new SparkMaxSim(followerController, DCMotor.getNEO(1));
    this.encoderSim = leaderSim.getRelativeEncoderSim();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  void shutdown() throws Exception {
    elevator.close();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void testSetSetpoint() {
    GameState stage = GameState.HUMAN_PLAYER_STATION;
    elevator.setSetpoint(stage);
    assertEquals(
        Constants.reefMap.get(stage).distance().magnitude(), elevator.setpoint.magnitude(), DELTA);
  }

  @Test
  void testAtSetpoint() {
    for (GameState stage : GameState.values()) {
      elevator.setSetpoint(stage);
      encoderSim.setPosition(
          elevator.heightToAngle(Constants.reefMap.get(stage).distance()).in(Rotations));
      elevator.periodic();
      assertTrue(elevator.atSetpoint());
    }
  }
}
