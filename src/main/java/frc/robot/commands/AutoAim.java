package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.stream.Collectors;
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
    Pose2d curPose = drive.getPose();
    var resultsOne = VisionConstants.aprilCamOne.getAllUnreadResults();
    var resultsTwo = VisionConstants.aprilCamTwo.getAllUnreadResults();

    if (!resultsOne.isEmpty() || !resultsTwo.isEmpty()) {
      var resultOne = resultsOne.get(resultsOne.size() - 1);
      var targetOne = resultOne.getBestTarget();
      var resultTwo = resultsTwo.get(resultsTwo.size() - 1);
      var targetTwo = resultTwo.getBestTarget();

      PhotonTrackedTarget target;
      if (targetOne != null && targetTwo != null) {
        target =
            targetOne.getPoseAmbiguity() < targetTwo.getPoseAmbiguity() ? targetOne : targetTwo;
      } else if (targetOne != null) {
        target = targetOne;
      } else if (targetTwo != null) {
        target = targetTwo;
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
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
