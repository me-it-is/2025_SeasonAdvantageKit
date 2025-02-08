package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Auto align to specified April Tag. */
public class AutoAim extends Command {
  private final Drive drive;

  public AutoAim(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    PhotonTrackedTarget target;
    Pose2d curPose = drive.getPose();
    var camOne = VisionConstants.aprilCamOne.getAllUnreadResults();
    var camTwo = VisionConstants.aprilCamTwo.getAllUnreadResults();

    if (!camOne.isEmpty() && !camTwo.isEmpty()) {
      var resultOne = camOne.get(camOne.size() - 1);
      var targetOne = resultOne.getBestTarget();
      var resultTwo = camTwo.get(camTwo.size() - 1);
      var targetTwo = resultTwo.getBestTarget();
      target = targetOne.getPoseAmbiguity() < targetTwo.getPoseAmbiguity() ? targetOne : targetTwo;
    } else if (!camOne.isEmpty()) {
      var resultOne = camOne.get(camOne.size() - 1);
      target = resultOne.getBestTarget();
    } else if (!camTwo.isEmpty()) {
      var resultTwo = camTwo.get(camTwo.size() - 1);
      target = resultTwo.getBestTarget();
    } else {
      target = null;
    }

    if (target != null) {
      List<Integer> key =
          VisionConstants.tagToHeight.keySet().stream()
              .filter(k -> k.contains(target.getFiducialId()))
              .collect(Collectors.toList())
              .get(0);
      double distance =
          PhotonUtils.calculateDistanceToTargetMeters(
              VisionConstants.kCameraHeight,
              VisionConstants.tagToHeight.get(key).in(Units.Meters),
              VisionConstants.kCameraPitchRadians,
              target.getPitch());

      // apply pid controller outputs to drivetrain controlling method
      double moveOutput =
          DriveConstants.translationController.calculate(
              distance, DriveConstants.TAG_DISTANCE.in(Units.Meters));
      Optional<Pose3d> tagPose =
          VisionConstants.aprilTagFieldLayout.getTagPose(target.getFiducialId());
      double moveOutput = 0.0;
      if (tagPose.isPresent()) {
        System.out.println("tag pose rot: " + tagPose.get().getRotation().toString());
        System.out.println("tag pose translation: " + tagPose.get().getTranslation().toString());
        double tagHeight = tagPose.get().getZ();
        double distance =
            PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.camChassisZOffset,
                tagHeight,
                VisionConstants.kCameraPitchRadians,
                target.getPitch());
        moveOutput =
            DriveConstants.translationController.calculate(
                distance, DriveConstants.TAG_DISTANCE.in(Units.Meters));
      }
      double turnOutput =
          DriveConstants.rotationController.calculate(
              -curPose.getRotation().getRadians(), target.getYaw());

      ChassisSpeeds speeds =
          new ChassisSpeeds(
              moveOutput * drive.getMaxLinearSpeedMetersPerSec(),
              0,
              turnOutput * drive.getMaxAngularSpeedRadPerSec());
      drive.runVelocity(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
