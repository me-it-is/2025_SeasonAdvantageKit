package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import org.photonvision.PhotonUtils;

public class AutoAimTest extends Command {
  private Drive drive;
  private Vision vision;
  private CommandXboxController controller;
  private double targetYaw;
  private double targetRange;
  private double angErr;
  private double transErr;

  public AutoAimTest(Drive drive, Vision vision, CommandXboxController controller) {
    this.drive = drive;
    this.vision = vision;
    this.controller = controller;
    this.targetYaw = 0.0;
    this.targetRange = 0.0;
    addRequirements(drive, vision);
  }

  @Override
  public void initialize() {
    angErr = VisionConstants.minAngError.in(Units.Degrees);
    transErr = VisionConstants.minTransError.in(Units.Meters);
  }

  @Override
  public void execute() {
    double forward =
        -controller.getLeftY() * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
    double strafe =
        -controller.getLeftX() * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
    double turn =
        -controller.getRightX() * DriveConstants.maxRotVelocity.in(Units.RadiansPerSecond);

    Pose3d target = null;
    Pose2d curPose = drive.getPose();
    List<Pose3d> bestTaggies = vision.getBestTags();
    if (bestTaggies != null) {
      if (bestTaggies.size() != 0) {
        System.out.println("pose is: " + bestTaggies.get(0));
      }
    }

    List<Pose3d> targets = vision.getBestTags();
    if (targets.size() != 0) {
      target = targets.get(targets.size() - 1);
    }
    if (target != null) {
      System.out.println("found tag");
      targetYaw = Radians.of(target.getRotation().getZ()).in(Units.Degrees);
      targetRange =
          PhotonUtils.calculateDistanceToTargetMeters(
              VisionConstants.camChassisZOffset, // Measured with a tape measure, or in CAD.
              0.22225, // From 2025 game manual for ID 22
              VisionConstants.kCameraPitchRadians, // Measured with a protractor, or in CAD.
              target.getRotation().getX());

      double curRot = curPose.getRotation().getDegrees();
      angErr = Math.abs(curRot - targetYaw);
      turn =
          DriveConstants.rotationController.calculate(curPose.getRotation().getDegrees(), targetYaw)
              * DriveConstants.maxRotVelocity.in(Units.RadiansPerSecond);
      transErr = Math.abs(targetRange - VisionConstants.tagDistSetpoint);
      forward =
          DriveConstants.translationController.calculate(
                  VisionConstants.tagDistSetpoint, targetRange)
              * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
      System.out.println("target range: " + targetRange);
      System.out.println("forward: " + forward + " strafe: " + strafe + " turn: " + turn);
      // Command drivetrain motors based on target speeds
      drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
    } else {
      System.out.println("no target found");
      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
  }

  @Override
  public boolean isFinished() {
    System.out.println("cur angle error in degrees: " + angErr);
    System.out.println("cur translation error in meters: " + transErr);
    if (angErr < VisionConstants.minAngError.in(Units.Degrees) && transErr < VisionConstants.minTransError.in(Units.Meters)) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
