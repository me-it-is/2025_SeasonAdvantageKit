// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode currentMode =
      Mode.REAL; // RobotBase.isReal() ? Mode.REAL : Mode.SIM; robot thinks it's fake wut

  // Phisical values of the robot
  public static final Mass ROBOT_MASS_KG = Pounds.of(138.308);
  // Converted form lbs in2 becuse wpi doesnt have a unit
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(0.0588);
  // esemate based of of this
  // https://www.researchgate.net/figure/Coefficient-of-friction-of-neoprene-rubber-with-different-part-materials_tbl1_223593062
  public static final double WHEEL_COF = 0.8;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum GameState {
    L1_SCORE,
    L2_SCORE,
    L3_SCORE,
    L4_SCORE,
    L2_ALGAE,
    L3_ALGAE,
    HUMAN_PLAYER_STATION,
    NONE,
  }

  public static record AngleAndDistance(Angle angle, Distance distance) {}

  public static Map<GameState, AngleAndDistance> reefMap =
      Map.of(
          GameState.L1_SCORE, new AngleAndDistance(Degrees.of(50), Inches.of(3.5)),
          GameState.L2_SCORE, new AngleAndDistance(Degrees.of(130), Inches.of(16.5)),
          GameState.L3_SCORE, new AngleAndDistance(Degrees.of(130), Inches.of(31)),
          GameState.L4_SCORE, new AngleAndDistance(Degrees.of(100), Inches.of(67.3)),
          GameState.L2_ALGAE, new AngleAndDistance(Degrees.of(58), Inches.of(3.5)),
          GameState.L3_ALGAE, new AngleAndDistance(Degrees.of(58), Inches.of(31)),
          GameState.HUMAN_PLAYER_STATION, new AngleAndDistance(Degrees.of(10), Inches.of(23)),
          GameState.NONE, new AngleAndDistance(Degrees.of(0), Inches.of(0)));

  public static class DriveConstants {
    public static final Distance chassisSize = Inches.of(34.24);
    public static final LinearVelocity maxTranslationSpeed = MetersPerSecond.of(30);
    public static final LinearAcceleration maxTranslationAcceleration =
        MetersPerSecondPerSecond.of(6);
    public static final AngularVelocity maxRotVelocity = RadiansPerSecond.of(4 * Math.PI);
    public static final AngularAcceleration maxRotAcceleration =
        RadiansPerSecondPerSecond.of(3 * Math.PI);
    public static final double DRIVE_DEADBAND = 0.02;
    public static final Distance TAG_DISTANCE = Meters.of(0.3);
    public static final PIDController translationController = new PIDController(0.5, 0.001, 0);
    public static final PIDController rotationController = new PIDController(0.03, 0.01, 0);

    public static final PIDController tipController = new PIDController(0.25, 0, 0.1);
    public static final double tipFF = 0.1;
    public static final Force tipDeadband = Newtons.of(3);
  }

  public static class ManipulatorConstants {
    public static final int PIVOT_ID = 5;
    public static final int ROLLER_ID = 6;
    public static final int LINE_BREAK_PORT = 3;

    public static final Angle fullRoll = Degrees.of(130);
    public static final Angle rotTolerance = Rotations.of(0.1);
    public static final double defaultRollerSpeed = 1.0;
    public static final Time defaultPickupTime = Seconds.of(1);
    public static final double gearRatio = 16;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kFF = 0.5;
  }

  public static class VisionConstants {
    public static final Distance kFieldWidth = Meters.of(16.54);
    public static final Distance kFieldHeight = Meters.of(8.229);

    public static final Distance kMinCamDistToTag = Meters.of(0.1);
    public static final Distance kMaxCamDistToTag = Meters.of(2.0);
    public static final Distance kMaxVertDisp = Meters.of(2.5);
    public static final double kMaxTagAmbiguity = 0.25;
    public static final Angle kRollBounds = Radians.of(0.2);
    public static final Angle kPitchBounds = Radians.of(0.2);

    private static final double halfOffset = DriveConstants.chassisSize.in(Units.Inches) / 2;
    public static final Distance kCamOneChassisXOffset = Inches.of(halfOffset);
    public static final Distance kCamOneChassisYOffset = Inches.of(halfOffset);
    public static final Distance kCamTwoChassisXOffset = Inches.of(-halfOffset);
    public static final Distance kCamTwoChassisYOffset = Inches.of(-halfOffset);
    public static final Distance KCamChassisZOffset = Inches.of(4.5);

    public static final Angle kMinAngError = Degrees.of(5);
    public static final Distance kMinTransError = Meters.of(0.05);

    // TODO placeholder offsets, need tuning
    public static final Distance kTagXOffset = Meters.of(0.1);
    public static final Distance kTagYOffset = Meters.of(0.2);

    public static final Distance tagDistSetpoint = Meters.of(0.1);
    public static final Angle kCameraPitch = Radians.of(Math.PI / 3);
    public static final Transform3d robotToCamOne =
        new Transform3d(
            new Translation3d(kCamOneChassisXOffset, kCamOneChassisYOffset, KCamChassisZOffset),
            new Rotation3d(0, kCameraPitch.in(Radians), 0));
    public static final Transform3d robotToCamTwo =
        new Transform3d(
            new Translation3d(
                kCamTwoChassisXOffset, kCamTwoChassisYOffset.unaryMinus(), KCamChassisZOffset),
            new Rotation3d(0, kCameraPitch.in(Radians), 0));

    public static final Matrix<N3, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.5, 0.5, Double.POSITIVE_INFINITY);

    public static AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  public static class ClimberConstants {
    public static final int kClimberMotorID = 2;
    public static final int kUpperLimSwitchId = 0;
    public static final int kLowerLimSwitchId = 1;
    public static final int kMidBeamBreakId = 2;
    public static final double kClimberMotorMult = 0.9;
  }

  public static class ElevatorConstants {
    public static final int sparkMaxCANId = 4;
    public static final int sparkMaxFollowerCANId = 3;
    public static final Distance maxHeight = Meters.of(1.72);
    public static final Distance levelOneSetpoint = Inches.of(3.93);
    public static final Distance levelTwoSetpoint = Inches.of(18.5);
    public static final Distance levelThreeSetpoint = Inches.of(46.3);
    public static final Distance levelFourSetpoint = Inches.of(67.3);
    public static final double deadReckoningSpeed = 0.1;
    public static final double deadRecogningDeadZone = 0.05;
    public static final double restInput = 0.02;
    public static final Distance setpointTolerance = Meters.of(0.1);

    public static double totalExtensionTime = maxHeight.in(Units.Meters) / deadReckoningSpeed;

    public static final Distance encoderOffset = Meters.of(0);

    public static class Config {
      public static final boolean inverted = false;
      public static final IdleMode idleMode = IdleMode.kBrake;

      // factor to make full extension 1 (1 / num rotations per full extension)
      public static final double positionConversionFactor = 1 / 17.893;

      public static final FeedbackSensor feedbackSensor = FeedbackSensor.kAbsoluteEncoder;
      public static final double pidP = 0.1;
      public static final double pidI = 0;
      public static final double pidD = 0;
      public static final double feedForward = 0.02;
    }
  }
}
