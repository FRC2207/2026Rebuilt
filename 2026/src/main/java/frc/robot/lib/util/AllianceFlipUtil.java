// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.util;

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
public class AllianceFlipUtil {
  public static enum CoordinateAxis {  
    X, Y
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(
        FieldConstants.fieldLength - translation.getX(), 
        FieldConstants.fieldWidth - translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double coordinate, CoordinateAxis axis) {
    if (shouldFlip()) {
      if (axis == CoordinateAxis.Y) {
        return FieldConstants.fieldWidth - coordinate;
      } else {  
        return FieldConstants.fieldLength - coordinate;
      }
    } else {
      return coordinate;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return rotation.plus(new Rotation2d(Math.PI));
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
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
    if (shouldFlip()) {
      return new Trajectory.State(
          state.timeSeconds,
          state.velocityMetersPerSecond,
          state.accelerationMetersPerSecondSq,
          new Pose2d(
              FieldConstants.fieldLength - state.poseMeters.getX(),
              FieldConstants.fieldWidth - state.poseMeters.getY(),
              state.poseMeters.getRotation().plus(new Rotation2d(Math.PI))),
          -state.curvatureRadPerMeter);
    } else {
      return state;
    }
  }

  private static boolean shouldFlip() {
    return DriverStation.getAlliance().get() == Alliance.Red;
  }
}