package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Class that uses drivetrain pose data and Photon camera measurements to calculate PID controller
 * output
 */
public class AutoAim {
  private PIDController movController;
  private PIDController rotController;
  private Supplier<Rotation2d> yaw;

  @AutoLogOutput private boolean enabled = false;

  // gets yaw from drivetrain
  public AutoAim(
      PIDController moveController, PIDController rotController, Supplier<Rotation2d> yaw) {
    this.movController = moveController;
    this.rotController = rotController;
    this.yaw = yaw;
  }

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
  }

  public boolean isEnabled() {
    return this.enabled;
  }

  public boolean isAimed() {
    return rotController.atSetpoint() || !this.enabled;
  }

  /** Returns result from camera with most accurate tag reading */
  public PhotonPipelineResult getLatestResult() {
    PhotonPipelineResult first = VisionConstants.aprilCamOne.getLatestResult();
    PhotonPipelineResult second = VisionConstants.aprilCamTwo.getLatestResult();
    double ambFirst = first.getBestTarget().poseAmbiguity;
    double ambSecond = second.getBestTarget().poseAmbiguity;
    return Math.min(ambFirst, ambSecond) == ambFirst ? first : second;
  }

  public double getChassisRotVelocity() {
    return rotController.calculate(yaw.get().getDegrees(), 0.0);
  }

  public double getChassisTranslateVelocity() {
    PhotonPipelineResult result = getLatestResult();
    double distance = 0.0;

    if (result.hasTargets()) {
      double pitch = result.getBestTarget().pitch;
      int id = result.getBestTarget().fiducialId;
      // calculate tag height based on id
      List<Integer> key =
          VisionConstants.tagToHeight.keySet().stream()
              .filter(k -> k.contains(id))
              .collect(Collectors.toList())
              .get(0);
      double tagHeight = VisionConstants.tagToHeight.get(key);
      distance =
          PhotonUtils.calculateDistanceToTargetMeters(
              VisionConstants.kCameraHeight, tagHeight, VisionConstants.kCameraPitchRadians, pitch);
    }

    double velocity = movController.calculate(distance, VisionConstants.tagDistSetpoint);
    return velocity;
  }
}
