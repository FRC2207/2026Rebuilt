// ...existing code...
package frc.robot.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.current.FieldConstants;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
public class AllianceRotationUtil {
  public static enum CoordinateAxis {  
    X, Y
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (translation == null) {
      // avoid returning null; return a safe origin translation
      translation = new Translation2d(0.0, 0.0);
    }
    if (shouldFlip()) {
      return new Translation2d(
        FieldConstants.fieldLength - translation.getX(), 
        FieldConstants.fieldWidth - translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips an X or Y coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(Double coordinate, CoordinateAxis axis) {
    // protect against nulls for convenience: if coordinate is null, treat as 0.0
    double coord = coordinate == null ? 0.0 : coordinate.doubleValue();
    if (axis == null) {
      // default to X axis if axis is not provided
      axis = CoordinateAxis.X;
    }
    if (shouldFlip()) {
      if (axis == CoordinateAxis.Y) {
        return FieldConstants.fieldWidth - coord;
      } else {  
        return FieldConstants.fieldLength - coord;
      }
    } else {
      return coord;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (rotation == null) {
      rotation = Rotation2d.fromDegrees(0.0);
    }
    if (shouldFlip()) {
      return rotation.plus(new Rotation2d(Math.PI));
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (pose == null) {
      pose = new Pose2d(); // origin, zero rotation
    }
    if (shouldFlip()) {
      return new Pose2d(
          FieldConstants.fieldLength - pose.getX(),
          FieldConstants.fieldWidth - pose.getY(),
          pose.getRotation().plus(new Rotation2d(Math.PI)));
    } else {
      return pose;
    }
  }

  /**
   * Flips a trajectory state to the correct side of the field based on the current alliance color.
   */
  public static Trajectory.State apply(Trajectory.State state) {
    if (state == null) {
      // return a safe no-op state rather than null
      return new Trajectory.State(
          0.0,
          0.0,
          0.0,
          new Pose2d(),
          0.0);
    }

    Pose2d pose = state.poseMeters == null ? new Pose2d() : state.poseMeters;
    Rotation2d rot = pose.getRotation() == null ? Rotation2d.fromDegrees(0.0) : pose.getRotation();

    if (shouldFlip()) {
      return new Trajectory.State(
          state.timeSeconds,
          state.velocityMetersPerSecond,
          state.accelerationMetersPerSecondSq,
          new Pose2d(
              FieldConstants.fieldLength - pose.getX(),
              FieldConstants.fieldWidth - pose.getY(),
              rot.plus(new Rotation2d(Math.PI))),
          // flip curvature sign for mirrored path
          -state.curvatureRadPerMeter);
    } else {
      return state;
    }
  }

  private static boolean shouldFlip() {
    try {
      Alliance a = DriverStation.getAlliance().get();
      return a == Alliance.Red;
    } catch (Exception e) {
      // DriverStation unavailable (unit tests / early init) — default to no flip
      return false;
    }
  }
}
// ...existing code...