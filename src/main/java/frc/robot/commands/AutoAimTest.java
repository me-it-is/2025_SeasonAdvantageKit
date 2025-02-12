package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoAimTest extends Command {
  private Drive drive;
  private Vision vision;
  private CommandXboxController controller;
  private double targetYaw;
  private double targetRange;

  public AutoAimTest(Drive drive, Vision vision, CommandXboxController controller) {
    this.drive = drive;
    this.vision = vision;
    this.controller = controller;
    this.targetYaw = 0.0;
    this.targetRange = 0.0;
    addRequirements(drive, vision);
  }

  @Override
  public void execute() {
    double forward =
        -controller.getLeftY() * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
    double strafe =
        -controller.getLeftX() * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
    double turn =
        -controller.getRightX() * DriveConstants.maxRotVelocity.in(Units.RadiansPerSecond);

    PhotonTrackedTarget target = null;
    Pose2d curPose = drive.getPose();
    // var camOne = vision.getCameraOne().getAllUnreadResults();
    List<Pose3d> bestTaggies = vision.getBestTags();
    if (bestTaggies != null) {
      if (bestTaggies.size() != 0) {
        System.out.println("pose is: " + bestTaggies.get(0));
      }
    }
    /*if (!camOne.isEmpty() && !camTwo.isEmpty()) {
      var resultOne = camOne.get(camOne.size() - 1);
      var resultTwo = camTwo.get(camTwo.size() - 1);
      if (resultOne.hasTargets() && resultTwo.hasTargets()) {
        var targetOne = resultOne.getBestTarget();
        var targetTwo = resultTwo.getBestTarget();
        target =
            targetOne.getPoseAmbiguity() < targetTwo.getPoseAmbiguity() ? targetOne : targetTwo;
      }
    } else if (!camOne.isEmpty()) {
      var resultOne = camOne.get(camOne.size() - 1);
      if (resultOne.hasTargets()) {
        target = resultOne.getBestTarget();
    }

    if (!camTwo.isEmpty()) {
      var resultTwo = camTwo.get(camTwo.size() - 1);
      if (resultTwo.hasTargets()) {
        target = resultTwo.getBestTarget();
      }
    }

    if (target != null) {
      System.out.println("target id is: " + target.getFiducialId());
      if (target.getFiducialId() == 22) {
        // Found Tag 22, record its information
        System.out.println("found tag 22");
        targetYaw = target.getYaw();
        targetRange =
            PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.camChassisZOffset, // Measured with a tape measure, or in CAD.
                0.22225, // From 2025 game manual for ID 22
                VisionConstants.kCameraPitchRadians, // Measured with a protractor, or in CAD.
                Units.Degrees.of(target.getPitch()).in(Units.Radians));
        turn =
            DriveConstants.rotationController.calculate(
                    curPose.getRotation().getDegrees(), targetYaw)
                * DriveConstants.maxRotVelocity.in(Units.RadiansPerSecond);
        forward =
            DriveConstants.translationController.calculate(
                    VisionConstants.tagDistSetpoint, targetRange)
                * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
      }
    } else {
      System.out.println("no target found");
    }*/

    System.out.println("target range: " + targetRange);
    System.out.println("forward: " + forward + " strafe: " + strafe + " turn: " + turn);
    // Command drivetrain motors based on target speeds
    drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
  }
}
