package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOReal implements VisionIO {

  protected final List<PhotonPoseEstimatorTuple> cameras;

  public VisionIOReal(List<PhotonPoseEstimatorNameTuple> cameras) {
    this.cameras =
        cameras.stream()
            .map(c -> new PhotonPoseEstimatorTuple(new PhotonCamera(c.camName()), c.estimator()))
            .toList();
    for (var cam : cameras) {
      cam.estimator().setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.visionResults =
        cameras.stream()
            .map(this::getResultsWithEstimator)
            .toArray(size -> new PhotonEstimatorAndResults[size]);
  }

  private PhotonEstimatorAndResults getResultsWithEstimator(PhotonPoseEstimatorTuple camAndEstim) {
    return new PhotonEstimatorAndResults(
        camAndEstim.estimator(), camAndEstim.photonCamera().getAllUnreadResults());
  }

  @Override
  public void close() {
    for (var cam : cameras) {
      cam.photonCamera().close();
    }
  }
}
