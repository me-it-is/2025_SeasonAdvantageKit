package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PhotonEstimatorAndResults;
import frc.robot.util.RobotMath;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
  private final Supplier<Pose2d> robotPose;

  /**
   * @param drivetrainUpdatePose Consumes vision PoseEstimates to update the Drivetrain's
   *     PoseEstimator
   */
  public Vision(
      Consumer<PoseEstimate> drivetrainUpdatePose, VisionIO visionIO, Supplier<Pose2d> robotPose) {
    this.visionIO = visionIO;
    this.dtUpdateEstimate = drivetrainUpdatePose;
    this.robotPose = robotPose;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(inputs);
    // Updates the Drivetrain PoesEstimator with both camera streams
    if (inputs.visionResults == null) return;
    List.of(inputs.visionResults).stream()
        .map(this::updateAngleAndGetEstimate)
        .flatMap(Optional::stream)
        .filter(Vision::isOnField)
        .filter(v -> pitchIsInBounds(v, kPitchBounds))
        .filter(v -> rollIsInBounds(v, kRollBounds))
        .map(Vision::clampToFloor)
        .map(v -> generatePoseEstimate(v, robotPose.get()))
        .forEach(
            (pose) -> {
              dtUpdateEstimate.accept(pose);
            }); // updates drivetrain swerve pose estimator with vision measurement
  }

  public List<TagTuple> getBestTags() {
    return List.of(inputs.visionResults).stream()
        .map(r -> r.results())
        .flatMap(rlist -> rlist.stream())
        .filter(PhotonPipelineResult::hasTargets)
        .map(r -> r.getBestTarget())
        .map(PhotonTrackedTarget::getFiducialId)
        .map(tagId -> new TagTuple(tagId, kAprilTagFieldLayout.getTagPose(tagId)))
        .toList();
  }

  private static final EstimateTuple clampToFloor(EstimateTuple estim) {
    var visEst = estim.visionEstimate;
    visEst =
        new EstimatedRobotPose(
            new Pose3d(visEst.estimatedPose.toPose2d()),
            visEst.timestampSeconds,
            visEst.targetsUsed,
            visEst.strategy);
    return estim;
  }

  private static PoseEstimate generatePoseEstimate(
      EstimateTuple estimateAndInfo, Pose2d currentPose) {
    final var stdDevs =
        kMultiTagStdDevs
            .times(Math.pow(averageDistance(estimateAndInfo, currentPose).in(Meters), 1.5))
            .times(1 / Math.pow(estimateAndInfo.visionEstimate.targetsUsed.size(), 2))
            .times(Math.pow(estimateAndInfo.ambiguity * 10, 4));
    return new PoseEstimate(estimateAndInfo.visionEstimate, stdDevs);
  }

  private static List<Distance> getDistances(EstimateTuple estimate, Pose2d currentPose) {
    return estimate.visionEstimate.targetsUsed.stream()
        .map(
            target ->
                RobotMath.distanceBetweenTranslations(
                    target.getBestCameraToTarget().getTranslation().toTranslation2d(),
                    currentPose.getTranslation()))
        .toList();
  }

  private static Distance averageDistance(EstimateTuple estimate, Pose2d currentPose) {
    List<Distance> distances = getDistances(estimate, currentPose);

    Distance averageDistance = Meters.zero();
    for (Distance dist : distances) {
      averageDistance = averageDistance.plus(dist);
    }
    return averageDistance.div(distances.size());
  }

  private static Distance maxDistance(EstimateTuple estimate, Pose2d currentPose) {
    List<Distance> distances = getDistances(estimate, currentPose);

    Distance maxDist = Meters.of(Double.POSITIVE_INFINITY);
    for (Distance dist : distances) {
      if (dist.lt(maxDist)) maxDist = dist;
    }
    return maxDist;
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

    latestResult.targets =
        latestResult.targets.stream()
            .filter(
                t -> t.bestCameraToTarget.getTranslation().getNorm() <= kMaxCamDistToTag.in(Meters))
            .filter(t -> t.poseAmbiguity <= kMaxTagAmbiguity)
            .toList();

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
  private static boolean averageDistanceIsInThreshold(
      EstimateTuple estimateAndInfo, Pose2d currentPose) {
    return RobotMath.measureWithinBounds(
        averageDistance(estimateAndInfo, currentPose), kMinCamDistToTag, kMaxCamDistToTag);
  }

  /** Is the robot on the field based on its current pose */
  private static boolean isOnField(Pose3d pose) {
    final var poseX = pose.getMeasureX();
    final var poseY = pose.getMeasureY();
    final var poseZ = pose.getMeasureZ();

    return RobotMath.measureWithinBounds(poseX, Meters.zero(), kFieldWidth)
        && RobotMath.measureWithinBounds(poseY, Meters.zero(), kFieldHeight)
        && RobotMath.measureWithinBounds(poseZ, kMaxVertDisp.unaryMinus(), kMaxVertDisp);
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
