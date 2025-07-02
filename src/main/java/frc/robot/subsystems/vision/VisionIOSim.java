package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.kAprilTagFieldLayout;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim extends VisionIOReal {
  private static VisionSystemSim sim;

  private final Supplier<Pose2d> poseSupplier;
  private final List<PhotonCameraSim> simCameras;

  public VisionIOSim(List<PhotonPoseEstimatorTuple> cameras, Supplier<Pose2d> robotPoseSupplier) {
    super(cameras);

    this.poseSupplier = robotPoseSupplier;

    // Initialize vision sim
    if (sim == null) {
      sim = new VisionSystemSim("main");
      sim.addAprilTags(kAprilTagFieldLayout);
    }

    simCameras = new ArrayList<>();
    var cameraProperties = new SimCameraProperties();
    for (var cam : cameras) {
      var simCam = new PhotonCameraSim(cam.photonCamera(), cameraProperties);
      simCameras.add(simCam);
      sim.addCamera(simCam, cam.estimator().getRobotToCameraTransform());
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    sim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
