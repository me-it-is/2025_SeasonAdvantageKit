package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PhotonEstimatorAndResults;
import frc.robot.util.RobotMath;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Predicate;
import org.photonvision.EstimatedRobotPose;

public class Vision extends SubsystemBase implements AutoCloseable {

  /** static class wrapping a pose estimate with standard derivations for position and rotation */
  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}
  /** A PoseEstimate and the associated PhotonCamera name and pose ambiguity */
  public static record EstimateTuple(EstimatedRobotPose visionEstimate, double ambiguity) {}

  /** Tag id and tag Optional<Pose3d> on field */
  public static record TagTuple(Integer tagId, Optional<Pose3d> tagPose) {}

  private VisionIO visionIO;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  private final Consumer<PoseEstimate> dtUpdateEstimate;

  /**
   * @param drivetrainUpdatePose Consumes vision PoseEstimates to update the Drivetrain's
   *     PoseEstimator
   */
  public Vision(Consumer<PoseEstimate> drivetrainUpdatePose, VisionIO visionIO) {
    this.visionIO = visionIO;
    this.dtUpdateEstimate = drivetrainUpdatePose;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(inputs);
    // Updates the Drivetrain PoesEstimator with both camera streams
    inputs
        .visionResults
        .map(this::updateAngleAndGetEstimate)
        .filter(Objects::nonNull)
        .flatMap(Optional::stream)
        .filter(Objects::nonNull)
        .filter(Vision::isOnField)
        /*.filter(Vision::maxDistanceIsInThreshold)
        .filter(Vision.isAmbiguityLess(VisionConstants.kMaxTagAmbiguity))
        .filter(v -> pitchIsInBounds(v, VisionConstants.kPitchBounds))
        .filter(v -> rollIsInBounds(v, VisionConstants.kRollBounds))*/
        .map(Vision::generatePoseEstimate)
        .forEach(
            (pose) -> {
              System.out.println("new vision pose estimate: " + pose);
              dtUpdateEstimate.accept(pose);
            }); // updates drivetrain swerve pose estimator with vision measurement
  }

  private static PoseEstimate generatePoseEstimate(EstimateTuple estimateAndInfo) {
    Distance maxDistance =
        Meters.of(
            estimateAndInfo.visionEstimate.targetsUsed.stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .max()
                .orElse(0.0));

    final var stdDevs =
        VisionConstants.kMultiTagStdDevs
            .times(maxDistance.in(Meters))
            .times(4 / Math.pow(estimateAndInfo.visionEstimate.targetsUsed.size(), 2));
    return new PoseEstimate(estimateAndInfo.visionEstimate, stdDevs);
  }

  /**
   * @param camToEstimator
   * @param results
   * @return
   */
  private Optional<EstimateTuple> updateAngleAndGetEstimate(
      PhotonEstimatorAndResults estimatorAndResults) {
    var results = estimatorAndResults.results();
    if (results.isEmpty()) {
      return Optional.empty();
    }

    final var latestResult = results.get(results.size() - 1);
    if (!latestResult.hasTargets()) {
      return Optional.empty();
    }

    final var estimatedPose = estimatorAndResults.estimator().update(latestResult);
    if (estimatedPose.isEmpty()) {
      return Optional.empty();
    }

    final var ambiguity = latestResult.getBestTarget().getPoseAmbiguity();
    return Optional.of(new EstimateTuple(estimatedPose.get(), ambiguity));
  }

  private static Predicate<EstimateTuple> isAmbiguityLess(double maxAmbiguity) {
    return estimateAndInfo ->
        estimateAndInfo.visionEstimate.targetsUsed.stream()
            .allMatch(trackedTarget -> trackedTarget.getPoseAmbiguity() < maxAmbiguity);
  }

  /**
   * Whether max distance to target from current pose falls in range, measured in meters, for
   * accurate readings
   */
  private static boolean maxDistanceIsInThreshold(EstimateTuple estimateAndInfo) {
    Distance maxDistance =
        Meters.of(
            estimateAndInfo.visionEstimate.targetsUsed.stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .max()
                .orElse(0.0));

    return RobotMath.measureWithinBounds(
        maxDistance, VisionConstants.kMinCamDistToTag, VisionConstants.kMaxCamDistToTag);
  }

  /** Is the robot on the field based on its current pose */
  private static boolean isOnField(Pose3d pose) {
    final var poseX = pose.getMeasureX();
    final var poseY = pose.getMeasureY();
    final var poseZ = pose.getMeasureZ();

    return RobotMath.measureWithinBounds(poseX, Meters.zero(), VisionConstants.kFieldWidth)
        && RobotMath.measureWithinBounds(poseY, Meters.zero(), VisionConstants.kFieldHeight)
        && RobotMath.measureWithinBounds(poseZ, Meters.zero(), VisionConstants.kMaxVertDisp);
  }

  /**
   * @param estTuple the EstimateTuple to check
   * @return true if the robot is on the field based on camera's pose data, false otherwise.
   * @implNote Should be used when camerareadings are available.
   */
  private static boolean isOnField(EstimateTuple estTuple) {
    return isOnField(estTuple.visionEstimate.estimatedPose);
  }

  /**
   * @param estTuple the {@link EstimateTuple} to check.
   * @return true if pitch in bounds, false otherwise.
   */
  private static boolean pitchIsInBounds(EstimateTuple estimateAndInfo, Angle pitchBounds) {
    final var estPoseX = estimateAndInfo.visionEstimate.estimatedPose.getRotation().getMeasureX();
    return RobotMath.measureWithinBounds(estPoseX, pitchBounds);
  }

  /**
   * @param estTuple the {@link EstimateTuple} to check.
   * @return true if roll is in bounds, false otherwise.
   */
  private static boolean rollIsInBounds(EstimateTuple estimateAndInfo, Angle rollBounds) {
    final var estPoseX = estimateAndInfo.visionEstimate.estimatedPose.getRotation().getMeasureX();
    return RobotMath.measureWithinBounds(estPoseX, rollBounds);
  }

  @Override
  public void close() {
    visionIO.close();
  }
}
