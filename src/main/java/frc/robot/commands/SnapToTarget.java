package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.kTargetPoses;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RobotMath;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Using current pose, goes to nearest known target based on field layout */
public class SnapToTarget extends Command {
  public static record TargetPose(Pose2d pose, Alliance alliance, boolean scoring, boolean right) {}

  private final Drive drive;

  public SnapToTarget(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d drivePose = drive.getPose();
    Pose2d scorePose = getClosestScoringPose(drivePose);
    // make path between start and end pose on the fly
    PathConstraints constraints =
        new PathConstraints(
            DriveConstants.kMaxPathSpeed,
            DriveConstants.kMaxTranslationAcceleration,
            DriveConstants.kMaxRotVelocity,
            DriveConstants.kMaxRotAcceleration);

    Logger.recordOutput("Vision/snaptoTargetPos", scorePose);

    Command pathFollow = AutoBuilder.pathfindToPose(scorePose, constraints);
    CommandScheduler.getInstance().schedule(pathFollow);
  }

  public Pose2d getClosestScoringPose(Pose2d drivePose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (!alliance.isPresent()) {
      return drivePose;
    }
    List<Pose2d> allPoses = kTargetPoses.stream().map(t -> t.pose()).toList();
    Logger.recordOutput("Vision/all target poses", allPoses.toArray(new Pose2d[allPoses.size()]));

    List<Pose2d> scrorringPoses =
        kTargetPoses.stream()
            .filter(t -> (t.alliance() == alliance.get()))
            .filter(t -> (t.scoring()))
            .map(t -> t.pose())
            .toList();
    Logger.recordOutput(
        "Vision/filterd target poses", scrorringPoses.toArray(new Pose2d[scrorringPoses.size()]));
    Pose2d minPose = scrorringPoses.get(0);
    Distance minDist = RobotMath.distanceBetweenPoses(drivePose, minPose);
    for (int i = 1; i < scrorringPoses.size(); i++) {
      Distance dist = RobotMath.distanceBetweenPoses(drivePose, scrorringPoses.get(i));
      if (dist.lt(minDist)) {
        minDist = dist;
        minPose = scrorringPoses.get(i);
      }
    }
    return minPose;
  }
}
