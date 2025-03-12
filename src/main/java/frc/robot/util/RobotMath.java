package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;

public final class RobotMath {
  /**
   * @param measure measure to check.
   * @param bounds bounds to check against.
   * @return true if -bounds < measure < bounds, false otherwise.
   */
  public static <U extends Unit> boolean measureWithinBounds(
      Measure<U> measure, Measure<U> bounds) {
    if (isMeasureNaN(measure)) {
      return false;
    }
    if (isMeasureNaN(bounds)) {
      return false;
    }
    return measure.lt(bounds) && measure.gt(bounds.unaryMinus());
  }

  /**
   * @param measure measure to check.
   * @param lower lower bound to check against.
   * @param upper upper bound to check against.
   * @return true if lower < measure < upper, false otherwise.
   */
  public static <U extends Unit> boolean measureWithinBounds(
      Measure<U> measure, Measure<U> lower, Measure<U> upper) {
    if (isMeasureNaN(measure)) {
      return false;
    }
    if (isMeasureNaN(lower)) {
      return false;
    }
    if (isMeasureNaN(upper)) {
      return false;
    }
    return measure.lt(upper) && measure.gt(lower);
  }

  /**
   * @param measure measure to get the absolute value of.
   * @param other measuere to get dist to.
   * @return abs of measuere1 - measuere2.
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M dist(M measure1, M measure2) {
    // No need for NaN checks as minus with retrun NaN if eather are NaN
    return abs((M) measure1.minus(measure2));
  }

  /**
   * @param measure measure to get the absolute value of.
   * @return the absulute value of measure with the zero point of the unit for that type.
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M abs(M measure) {
    if (isMeasureNaN(measure)) {
      // NaN's propagate
      return (M) measure.baseUnit().of(Double.NaN);
    }
    return measure.gte((M) measure.unit().zero())
        ? measure
        : (M) ((M) measure.unit().zero()).minus(measure);
  }

  public static <U extends Unit> boolean isWithinTolerance(
      Measure<U> measure1, Measure<U> measure2, Measure<U> tol) {
    if (isMeasureNaN(measure1)) {
      return false;
    }
    if (isMeasureNaN(measure2)) {
      return false;
    }
    if (isMeasureNaN(tol)) {
      return false;
    }
    return dist(measure1, measure2).lte(tol);
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit> boolean approxZero(Measure<U> measure) {
    return approxZero(measure, (Measure<U>) measure.unit().of(1e-6));
  }

  public static <U extends Unit> boolean approxZero(Measure<U> measure, Measure<U> tol) {
    if (isMeasureNaN(measure)) {
      return false;
    }
    return abs(measure).lte(tol);
  }

  public static <U extends Unit> boolean isMeasureNaN(Measure<U> measure) {
    return Double.isNaN(measure.baseUnitMagnitude());
  }

  public static Distance distanceBetweenTranslations(Translation3d trans1, Translation3d trans2) {
    return Meters.of(trans1.getDistance(trans2));
  }

  public static Distance distanceBetweenTranslations(Translation2d trans1, Translation2d trans2) {
    return distanceBetweenTranslations(new Translation3d(trans1), new Translation3d(trans2));
  }

  public static Distance distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
    return distanceBetweenTranslations(pose1.getTranslation(), pose2.getTranslation());
  }

  /**
   * @return 1 if b is true, -1 otherwise
   */
  public static int signBool(boolean b) {
    return b ? 1 : -1;
  }
}
