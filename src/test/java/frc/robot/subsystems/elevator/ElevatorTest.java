package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
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
  TalonFX leaderController;
  TalonFX followerController;
  TalonFXSimState leaderSim;
  TalonFXSimState followerSim;
  Distance setpoint = Constants.reefMap.get(GameState.NONE).distance();
  double DELTA = 1e-2;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 1);
    this.leaderController = new TalonFX(ElevatorConstants.kTalonLeaderCANId, "blinky");
    this.followerController = new TalonFX(ElevatorConstants.kTalonFollowerCANId, "blinky");
    this.elevator = new Elevator(leaderController, followerController);
    this.leaderSim = new TalonFXSimState(leaderController);
    this.followerSim = new TalonFXSimState(followerController);
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
      leaderSim.setRawRotorPosition(
          elevator.heightToAngle(Constants.reefMap.get(stage).distance()));
      elevator.periodic();
      elevator.preformBlockingRefereshOnAllSignals();
      assertTrue(elevator.atSetpoint());
    }
  }
}
