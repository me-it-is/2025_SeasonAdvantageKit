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

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ManipulatorConstants.*;
import static frc.robot.util.GetAliance.getAllianceBoolean;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.State;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SnapToTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.climber.ClimberIOSparkSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorIOTalonFXSim;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOSparkMax;
import frc.robot.subsystems.manipulator.ManipulatorIOSparkMaxSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.RobotMath;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Manipulator manipulator;
  private final Climber climber;
  private final Elevator elevator;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);
  private final CommandXboxController testController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Pose2d> startPoseLoc = new SendableChooser<>();

  private SwerveDriveSimulation driveSimulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight));

        manipulator =
            new Manipulator(
                new ManipulatorIOSparkMax(
                    new SparkMax(ManipulatorConstants.kPivotId, MotorType.kBrushless),
                    new SparkMax(ManipulatorConstants.kRollerId, MotorType.kBrushless)));

        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    new TalonFX(ElevatorConstants.kTalonLeaderCANId, ElevatorConstants.canBus),
                    new TalonFX(ElevatorConstants.kTalonFollowerCANId, ElevatorConstants.canBus)));

        climber =
            new Climber(
                new ClimberIOSpark(
                    new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless)));

        vision =
            new Vision(
                drive::updateEstimates,
                new VisionIOReal(VisionConstants.estimAndCam),
                drive::getPose);
        break;

      case SIM:
        SimulatedArena.getInstance().overrideSimulationTimings(Seconds.of(Constants.kDt), 25);
        driveSimulation =
            new SwerveDriveSimulation(Constants.DriveConstants.mapleSimConfig, Constants.startPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()) {},
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));

        manipulator =
            new Manipulator(
                new ManipulatorIOSparkMaxSim(
                    new SparkMax(ManipulatorConstants.kPivotId, MotorType.kBrushless),
                    new SparkMax(ManipulatorConstants.kRollerId, MotorType.kBrushless)));

        elevator =
            new Elevator(
                new ElevatorIOTalonFXSim(
                    new TalonFX(ElevatorConstants.kTalonLeaderCANId, ElevatorConstants.canBus),
                    new TalonFX(ElevatorConstants.kTalonFollowerCANId, ElevatorConstants.canBus),
                    SimulatedBattery::getBatteryVoltage));

        climber = new Climber(new ClimberIOSparkSim());

        SimulatedBattery.addElectricalAppliances(elevator::getTotalCurrent);

        vision =
            new Vision(
                drive::updateEstimates,
                new VisionIOSim(
                    VisionConstants.estimAndCam, driveSimulation::getSimulatedDriveTrainPose),
                drive::getPose);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        manipulator = new Manipulator(new ManipulatorIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        climber = new Climber(new ClimberIO() {});
        vision = new Vision(drive::updateEstimates, new VisionIO() {}, drive::getPose);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines

    Command drive1m = AutoBuilder.buildAuto("drive 1m");
    testController.x().whileTrue(drive.driveSysIdQuasistatic(kForward));
    testController.b().whileTrue(drive.driveSysIdQuasistatic(kReverse));
    testController.y().whileTrue(drive.driveSysIdDynamic(kForward));
    testController.a().whileTrue(drive.driveSysIdDynamic(kReverse));
    testController.povUp().whileTrue(drive.turnSysIdQuasistatic(kForward));
    testController.povDown().whileTrue(drive.turnSysIdQuasistatic(kReverse));
    testController.povLeft().whileTrue(drive.turnSysIdDynamic(kForward));
    testController.povRight().whileTrue(drive.turnSysIdDynamic(kReverse));

    testController.leftTrigger().whileTrue(characterizeElevatorQuasistatic(kForward));
    testController.leftBumper().whileTrue(characterizeElevatorQuasistatic(kReverse));
    testController.rightTrigger().whileTrue(characterizeElevatorDynamic(kForward));
    testController.rightBumper().whileTrue(characterizeElevatorDynamic(kReverse));

    // testController
    //     .povLeft()
    //     .whileTrue(characterizeElevator(Direction.kForward))
    //     .onFalse((runOnce(() -> drive.stop(), drive)));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)", drive.driveSysIdQuasistatic(kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)", drive.driveSysIdQuasistatic(kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.driveSysIdDynamic(kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.driveSysIdDynamic(kReverse));

    configureAutos();
    // Configure the button bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    configureButtonBindings();

    startPoseLoc.setDefaultOption("default", new Pose2d());
    // account for alliance (mirror x for red)
    startPoseLoc.addOption(
        "left",
        new Pose2d(
            new Translation2d(
                getAllianceBoolean() ? VisionConstants.kFieldHeight.in(Units.Meters) : 0,
                DriveConstants.kChassisSize.in(Units.Meters) / 2),
            new Rotation2d()));

    startPoseLoc.addOption(
        "center",
        new Pose2d(
            new Translation2d(
                VisionConstants.kFieldHeight.in(Units.Meters) / 2,
                DriveConstants.kChassisSize.in(Units.Meters) / 2),
            new Rotation2d()));

    startPoseLoc.addOption(
        "right",
        new Pose2d(
            new Translation2d(
                getAllianceBoolean() ? 0 : VisionConstants.kFieldHeight.in(Units.Meters),
                DriveConstants.kChassisSize.in(Units.Meters) / 2),
            new Rotation2d()));
    startPoseLoc.setDefaultOption("constant start", Constants.startPose);
    drive.setPose(startPoseLoc.getSelected());
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // SimulatedArena.getInstance().physicsWorld.setGravity(PhysicsWorld.EARTH_GRAVITY);
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  private void configureAutos() {
    NamedCommands.registerCommand(
        "score", moveToState(GameState.L4_SCORE, true).andThen(rollerAction(false)));
    NamedCommands.registerCommand(
        "hps pickup",
        moveToState(GameState.HUMAN_PLAYER_STATION, true).andThen(rollerAction(true)));
    NamedCommands.registerCommand(
        "remove algae", moveToState(GameState.L2_ALGAE, true).andThen(rollerAction(true)));

    autoChooser.addDefaultOption("top leave", AutoBuilder.buildAuto("top leave"));
    // autoChooser.addOption(
    //    "middle leave single score", AutoBuilder.buildAuto("middle leave single score"));
    autoChooser.addOption("middle leave", AutoBuilder.buildAuto("middle leave"));
    autoChooser.addOption("bottom leave", AutoBuilder.buildAuto("bottom leave"));
    autoChooser.addOption("top leave single score", AutoBuilder.buildAuto("top reef single score"));
    autoChooser.addOption("top leave double score", AutoBuilder.buildAuto("top reef double score"));
    autoChooser.addOption(
        "bottom leave single score", AutoBuilder.buildAuto("bottom reef single score"));
    autoChooser.addOption(
        "bottom leave double score", AutoBuilder.buildAuto("bottom reef double score"));
    autoChooser.addOption("top remove algae", AutoBuilder.buildAuto("top remove algae"));
    autoChooser.addOption("bottom remove algae", AutoBuilder.buildAuto("bottom remove algae"));
    autoChooser.addOption("drive 1m", AutoBuilder.buildAuto("drive 1m"));
  }

  public void resetPose() {
    drive.resetGyro();
    Command autoCommand = autoChooser.get();
    if (autoCommand instanceof PathPlannerAuto auto) {
      Pose2d startPose = auto.getStartingPose();
      drive.setPose(startPose);
    } else {
      System.out.println("No PathPlanner auto selected");
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -MathUtil.applyDeadband(controller.getLeftY(), DriveConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(-controller.getRightX(), DriveConstants.kDriveDeadband),
            controller.leftTrigger()));
    // Rotate and translate to closest April Tag based on tag odometry
    controller.b().whileTrue(new AutoAim(drive, vision, controller));

    // Automatically align to April Tag based on pose data
    controller.y().onTrue(new SnapToTarget(drive));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when A button is pressed
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // intake coral
    opController.rightBumper().onTrue(rollerAction(true));

    // outtake coral
    opController.leftBumper().onTrue(rollerAction(false));

    /*new Trigger(() -> (Math.abs(opController.getLeftY())) > 0.5)
    .whileTrue(
        runOnce(
            () -> elevator.voltageDrive(Volts.of(2 * Math.signum(opController.getLeftY()))),
            elevator))
    .onFalse(runOnce(() -> elevator.setUseVoltageControl(false), elevator));*/
    /*new Trigger(() -> (opController.getLeftY()) > 0.5 && elevator.isWithinUpperBounds())
        .whileTrue(runOnce(() -> characterizeElevatorQuasistatic(Direction.kForward), elevator))
        .onFalse(runOnce(elevator::stop, elevator));
    new Trigger(() -> (opController.getLeftY()) < -0.5 && elevator.isWithinLowerBounds())
        .whileTrue(runOnce(() -> characterizeElevatorQuasistatic(Direction.kReverse), elevator))
        .onFalse(runOnce(elevator::stop, elevator));*/

    opController.povLeft().onTrue(moveToState(GameState.L2_ALGAE, false));
    opController.povRight().onTrue(moveToState(GameState.L3_ALGAE, false));

    opController.a().onTrue(moveToState(GameState.L1_SCORE, false));
    opController.b().onTrue(moveToState(GameState.L2_SCORE, false));
    opController.y().onTrue(moveToState(GameState.L3_SCORE, false));
    opController.x().onTrue(moveToState(GameState.L4_SCORE, false));

    // human player station intake
    // opController.povUp().onTrue(moveToState(GameState.HUMAN_PLAYER_STATION, false));
    // opController.povUp().whileTrue(run(() -> manipulator.move(true), manipulator));
    opController
        .povUp()
        .onTrue(
            run(() -> manipulator.setAngle(GameState.L2_SCORE), manipulator)
                .withTimeout(0.5)
                .andThen(runOnce(() -> manipulator.stopRollers())));

    opController
        .povDown()
        .onTrue(
            runOnce(() -> climber.moveToSetpoint(State.MID), climber).until(climber::atSetpoint));

    opController
        .leftTrigger()
        .whileTrue(runOnce(() -> climber.run(true), climber))
        .onFalse(runOnce(climber::stop));

    opController
        .rightTrigger()
        .whileTrue(runOnce(() -> climber.run(false), climber))
        .onFalse(runOnce(climber::stop));
  }

  /* Move to correct elevator height, pivot angle, and spin manipulator rollers to counteract the force applyd on the coral by spinning the manipulator */
  private Command moveToState(GameState state, boolean auto) {
    return parallel(
        runOnce(() -> elevator.setSetpoint(state), elevator),
        runOnce(() -> manipulator.setAngle(state), manipulator),
        waitUntil(() -> manipulator.atSetpoint()).andThen(manipulator::stopRollers),
        waitUntil(elevator::atSetpoint));
  }

  private Command rollerAction(boolean forward) {
    return sequence(
        runOnce(() -> manipulator.spinRollers(forward), manipulator),
        waitUntil(() -> (forward ? !manipulator.hasCoral() : manipulator.hasCoral())),
        runOnce(manipulator::stopRollers, manipulator));
  }

  private Command characterizeElevatorQuasistatic(Direction dir) {
    SysIdRoutine routine =
        new SysIdRoutine(
            ElevatorConstants.sysIdConfig,
            new SysIdRoutine.Mechanism(elevator::voltageDrive, null, elevator));
    return routine.quasistatic(dir);
  }

  private Command characterizeElevatorDynamic(Direction dir) {
    SysIdRoutine routine =
        new SysIdRoutine(
            ElevatorConstants.sysIdConfig,
            new SysIdRoutine.Mechanism(elevator::voltageDrive, null, elevator));

    return routine.dynamic(dir);
  }

  private Command characterizeElevatorFull() {
    return sequence(
        characterizeElevatorQuasistatic(Direction.kForward)
            .until(
                () -> elevator.getElevatorHeight().gte(ElevatorConstants.kMaxHeight.times(0.85))),
        characterizeElevatorQuasistatic(Direction.kReverse)
            .until(
                () -> elevator.getElevatorHeight().lte(ElevatorConstants.kMaxHeight.times(0.15))),
        waitSeconds(5),
        characterizeElevatorDynamic(Direction.kForward)
            .until(
                () -> elevator.getElevatorHeight().gte(ElevatorConstants.kMaxHeight.times(0.85))),
        characterizeElevatorQuasistatic(Direction.kReverse)
            .until(
                () -> elevator.getElevatorHeight().lte(ElevatorConstants.kMaxHeight.times(0.15))));
  }

  public void resetSubsystems() {
    elevator.zeroElevator();
  }

  public void logMechanismForAScopeDisplay() {
    Distance height = elevator.getElevatorHeight();
    Distance elevatorCarriageHeight =
        (Distance) RobotMath.clamp(height, Meters.zero(), kCarriageMaxHeight);
    Distance elevatorStage2Height =
        (Distance)
            RobotMath.clamp(height.minus(kCarriageRideDist), Meters.zero(), kStage2MaxHeight);
    Distance elevatorStage1Height =
        (Distance)
            RobotMath.clamp(
                height.minus(kCarriageRideDist.plus(kStage2RideDist)),
                Meters.zero(),
                kStage1MaxHeight);
    Logger.recordOutput("Elevator/stage1Height", elevatorStage1Height.in(Meters));
    Logger.recordOutput("Elevator/stage2Height", elevatorStage2Height.in(Meters));
    Logger.recordOutput("Elevator/carriageHeight", elevatorCarriageHeight.in(Meters));
    Logger.recordOutput(
        "MechanismLocations",
        new Pose3d[] {
          new Pose3d(
              kStage1StartPos.getX(),
              kStage1StartPos.getY(),
              kStage1StartPos.getZ() + (elevatorStage1Height.in(Meters)),
              kStage1StartPos.getRotation()),
          new Pose3d(
              kStage2StartPos.getX(),
              kStage2StartPos.getY(),
              kStage2StartPos.getZ() + (elevatorStage2Height.in(Meters)),
              kStage2StartPos.getRotation()),
          new Pose3d(
              kCarriageStartPos.getX(),
              kCarriageStartPos.getY(),
              kCarriageStartPos.getZ() + (elevatorCarriageHeight.in(Meters)),
              kCarriageStartPos.getRotation()),
          new Pose3d(
              kStartingPose.getX(),
              kStartingPose.getY(),
              kStartingPose.getZ() + (elevatorCarriageHeight.in(Meters)),
              kStartingPose
                  .getRotation()
                  .rotateBy(
                      new Rotation3d(
                          0, -manipulator.getManipulatorAngle().in(Radians) - (Math.PI / 2), 0)))
        });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
