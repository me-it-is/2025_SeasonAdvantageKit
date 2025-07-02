package frc.robot.subsystems.vision;

import java.util.List;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  /** A PhotonCamera and its corresponding PhotonPoseEstimator */
  public static record PhotonPoseEstimatorTuple(
      PhotonCamera photonCamera, PhotonPoseEstimator estimator) {}
  /** A Estemator and its results */
  public static record PhotonEstimatorAndResults(
      PhotonPoseEstimator estimator, List<PhotonPipelineResult> results) {}

  @AutoLog
  public class VisionIOInputs {
    Stream<PhotonEstimatorAndResults> visionResults;
  }

  default void updateInputs(VisionIOInputs inputs) {}
  ;

  default void close() {}
  ;
}
