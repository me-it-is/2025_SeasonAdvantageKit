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
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode currentMode =
      Mode.REAL; // RobotBase.isReal() ? Mode.REAL : Mode.SIM; robot thinks it's fake wut
  public static final Alliance deafaultAlliance = Alliance.Blue;

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
    public static final Distance kChassisSize = Inches.of(34.24);
    public static final LinearVelocity kMaxTranslationSpeed = MetersPerSecond.of(30);
    public static final LinearAcceleration kMaxTranslationAcceleration =
        MetersPerSecondPerSecond.of(6);
    public static final AngularVelocity kMaxRotVelocity = RadiansPerSecond.of(4 * Math.PI);
    public static final AngularAcceleration kMaxRotAcceleration =
        RadiansPerSecondPerSecond.of(3 * Math.PI);
    public static final double kDriveDeadband = 0.02;
    public static final Distance kTagDistance = Meters.of(0.3);
    public static final PIDController kTranslationController = new PIDController(0.5, 0.001, 0);
    public static final PIDController kRotationController = new PIDController(0.03, 0.01, 0);

    public static final PIDController tipController = new PIDController(0.25, 0, 0.1);
    public static final double tipFF = 0.1;
    public static final Force tipDeadband = Newtons.of(3);
  }

  public static class ManipulatorConstants {
    public static final int kPivotId = 5;
    public static final int kRollerId = 6;
    public static final int kLineBreakPort = 3;

    public static final Angle kFullRoll = Degrees.of(130);
    public static final Angle kRotTolerance = Rotations.of(0.1);
    public static final double kDefaultRollerSpeed = 1.0;
    public static final Time kDefaultPickupTime = Seconds.of(1);
    public static final double kGearRatio = 16;
    public static final double kP = 0.7;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kFF = 0.4;
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

    private static final double halfOffset = DriveConstants.kChassisSize.in(Units.Inches) / 2;
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
    public static final Transform3d kRobotToCamOne =
        new Transform3d(
            new Translation3d(kCamOneChassisXOffset, kCamOneChassisYOffset, KCamChassisZOffset),
            new Rotation3d(0, kCameraPitch.in(Radians), 0));
    public static final Transform3d kRobotToCamTwo =
        new Transform3d(
            new Translation3d(
                kCamTwoChassisXOffset, kCamTwoChassisYOffset.unaryMinus(), KCamChassisZOffset),
            new Rotation3d(0, kCameraPitch.in(Radians), 0));

    public static final Matrix<N3, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.5, 0.5, Double.POSITIVE_INFINITY);

    public static final AprilTagFieldLayout kAprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  public static class ClimberConstants {
    public static final int kClimberMotorID = 2;
    public static final int kUpperLimSwitchId = 0;
    public static final int kLowerLimSwitchId = 1;
    public static final int kMidBeamBreakId = 2;
    public static final double kClimberMotorMult = 0.9;

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static enum State {
      TOP,
      BOTTOM,
      MID,
    }

    public static final Map<State, Angle> stateMap =
        Map.of(
            State.TOP,
            Rotations.of(0),
            State.MID,
            Rotations.of(0.2),
            State.BOTTOM,
            Rotations.of(0.25));
    public static final Angle setpointTolerance = Rotations.of(0.05);
  }

  public static class ElevatorConstants {
    public static final int kTalonLeaderCANId = 4;
    public static final int kSparkMaxFollowerCANId = 3;
    public static final Distance kMaxHeight = Meters.of(1.72);
    public static final double kDeadReckoningSpeed = 0.1;
    public static final double kDeadRecogningDeadZone = 0.05;
    public static final double kRestInput = 0.02;
    public static final Distance kSetpointTolerance = Meters.of(0.1);
    public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(5);
    public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(5);

    public static Time totalExtensionTime =
        Seconds.of(kMaxHeight.in(Units.Meters) / kDeadReckoningSpeed);

    public static final Distance kEncoderOffset = Meters.of(0);

    public static final boolean kIsInverted = true;

    // factor to make full extension 1 (1 / num rotations per full extension)
    public static final double kRotsPerFullExtension = 17.893;
    public static final double kPositionConversionFactor = 1 / kRotsPerFullExtension;

    public static final FeedbackSensor feedbackSensor = FeedbackSensor.kAbsoluteEncoder;
    public static final double kG = 0.5; // volts to overcome gravity
    public static final double kP = 0.5; // volts per rotations off
    public static final double kI = 0;
    public static final double kD = 0.1; // volts per rotations per second off
    // TODO have good constants

    public static Slot0Configs getSharedConfig() {
      Slot0Configs config = new Slot0Configs();
      config.kG = kG;
      config.kP = kP;
      config.kI = kI;
      config.kD = kD;
      config.GravityType = GravityTypeValue.Elevator_Static;
      return config;
    }
  }
}
