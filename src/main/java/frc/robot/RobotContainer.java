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

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ManipulatorConstants.LINE_BREAK_PORT;
import static frc.robot.Constants.ManipulatorConstants.PIVOT_ID;
import static frc.robot.Constants.ManipulatorConstants.ROLLER_ID;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameState;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SnapToTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Manipulator manipulator;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Pose2d> startPoseLoc = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Monologue.setupMonologue(this, "Robot", false, false);
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
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
        break;
    }

    vision = new Vision(drive::updateEstimates);
    manipulator =
        new Manipulator(
            new SparkMax(PIVOT_ID, MotorType.kBrushless),
            new SparkMax(ROLLER_ID, MotorType.kBrushless),
            new DigitalInput(LINE_BREAK_PORT));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureAutos();
    // Configure the button bindings
    configureButtonBindings();

    startPoseLoc.setDefaultOption("default", new Pose2d());
    // account for alliance (mirror x for red)
    startPoseLoc.addOption(
        "left",
        new Pose2d(
            new Translation2d(
                DriverStation.getAlliance().get() == Alliance.Blue
                    ? VisionConstants.kFieldHeight.in(Units.Meters)
                    : 0,
                DriveConstants.chassisSize.in(Units.Meters) / 2),
            new Rotation2d()));

    startPoseLoc.addOption(
        "center",
        new Pose2d(
            new Translation2d(
                VisionConstants.kFieldHeight.in(Units.Meters) / 2,
                DriveConstants.chassisSize.in(Units.Meters) / 2),
            new Rotation2d()));

    startPoseLoc.addOption(
        "right",
        new Pose2d(
            new Translation2d(
                DriverStation.getAlliance().get() == Alliance.Blue
                    ? 0
                    : VisionConstants.kFieldHeight.in(Units.Meters),
                DriveConstants.chassisSize.in(Units.Meters) / 2),
            new Rotation2d()));
    drive.setPose(startPoseLoc.getSelected());
  }

  private void configureAutos() {
    NamedCommands.registerCommand("test", print("test"));
    autoChooser.addOption("left leave", AutoBuilder.buildAuto("left leave"));
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
            () -> MathUtil.applyDeadband(controller.getLeftY(), DriveConstants.DRIVE_DEADBAND),
            () -> MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.DRIVE_DEADBAND),
            () -> MathUtil.applyDeadband(-controller.getRightX(), DriveConstants.DRIVE_DEADBAND)));

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
    opController.rightBumper().onTrue(manipulator.spinRollers(true).until(manipulator::hasCoral));

    // outtake coral
    opController
        .leftBumper()
        .onTrue(
            manipulator
                .spinRollers(false)
                .withTimeout(ManipulatorConstants.defaultPickupActionTime));

    // intake and release algae
    new Trigger(() -> (Math.abs(opController.getRawAxis(1)) > 0.5))
        .onTrue(
            pickupAction(
                GameState.L2_ALGAE, Math.signum(opController.getRawAxis(1)) == 1.0 ? true : false));

    opController.a().onTrue(pickupAction(GameState.L1_SCORE, true));
    opController.b().onTrue(pickupAction(GameState.L2_SCORE, true));
    opController.y().onTrue(pickupAction(GameState.L3_SCORE, true));
    opController.x().onTrue(pickupAction(GameState.L4_SCORE, true));

    // human player station intake
    opController.povUp().onTrue(pickupAction(GameState.HUMAN_PLAYER_STATION, false));
  }

  // TODO add elevator movement (extend then retract once finished) once subsystems are tested and
  private Command pickupAction(GameState state, boolean eject) {
    return manipulator
        .setAngle(state)
        .until(() -> manipulator.atAngle(state))
        .andThen(
            sequence(
                manipulator
                    .spinRollers(eject)
                    .withTimeout(ManipulatorConstants.defaultPickupActionTime),
                manipulator.stopRollers()));
  }

  public void driveTipCorrect() {
    drive.runVelocity(drive.calculateTipCorrection());
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
