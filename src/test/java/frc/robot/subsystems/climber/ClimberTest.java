package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.State;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ClimberTest {
  SparkMax motorController;
  Climber climber;
  SparkMaxSim motorSim;
  SparkAbsoluteEncoderSim encoderSim;
  CommandScheduler commandScheduler;
  static final double DELTA = 1e-2;
  boolean hasInterrupted;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 1);
    this.motorController = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    this.climber = new Climber(motorController);
    this.motorSim = new SparkMaxSim(motorController, DCMotor.getNEO(1));
    this.encoderSim = motorSim.getAbsoluteEncoderSim();
    this.commandScheduler = CommandScheduler.getInstance();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  void shutdown() throws Exception {
    climber.close();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void testRunForwards() {
    climber.run(true);
    assertEquals(ClimberConstants.kClimberMotorMult, motorSim.getSetpoint(), DELTA);
  }

  @Test
  void testRunBackwards() {
    climber.run(false);
    assertEquals(ClimberConstants.kClimberMotorMult * -1, motorSim.getSetpoint(), DELTA);
  }

  @Test
  void testSetPosition() {
    double position = 1.0;
    motorSim.getAbsoluteEncoderSim().setPosition(position);
    assertEquals(position, climber.getPosition().in(Rotations), DELTA);
  }

  @Test
  void testAtPostion() {
    climber.setpoint = Rotations.of(0.2);
    encoderSim.setPosition(0.1);
    assertFalse(encoderSim.getPosition() == 0.2);
    assertFalse(climber.atSetpoint());
    encoderSim.setPosition(0.2);
    assertEquals(0.2, encoderSim.getPosition(), DELTA);
    assertTrue(climber.atSetpoint());
  }

  @Test
  void testMoveToSetpoint() {
    this.hasInterrupted = false;
    State stage = State.MID;
    Command cmd =
        climber.untilState(
            stage,
            climber
                .setReferenceCommand(stage)
                .ignoringDisable(true)
                .handleInterrupt(
                    () -> {
                      this.hasInterrupted = true;
                    }));

    assertEquals(ClimberConstants.stateMap.get(stage), climber.setpoint);
    CommandScheduler.getInstance().schedule(cmd);
    CommandScheduler.getInstance().run();

    assertEquals(climber.setpoint.in(Rotations), motorSim.getSetpoint(), DELTA);
    encoderSim.setPosition(climber.setpoint.in(Rotations));
    assertTrue(climber.atSetpoint());

    CommandScheduler.getInstance().run();
    assertTrue(cmd.isFinished());
    assertTrue(this.hasInterrupted);
  }
}
