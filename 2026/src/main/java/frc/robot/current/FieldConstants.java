// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.current;

import static edu.wpi.first.apriltag.AprilTagFields.k2026RebuiltWelded;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(651.22);
  public static double fieldWidth = Units.inchesToMeters(317.7);

  /** The centers of the blue alliance elements */
  public static final class Elements {
    // Blue alliance elements
    public static Translation2d blueOutpost = new Translation2d(
        0.35,
        0.68); // 26.22 = 0.67 meters

    public static Translation2d blueDepot = new Translation2d(
      1,
      5.9);

    public static Translation2d blueHub = new Translation2d(
      4.62,
      4
    );

    public static Pose2d blueHubPose = new Pose2d(blueHub, new Rotation2d(0.0));
 
    // Red alliance elements
    public static Translation2d redOutpost = new Translation2d(
      16.19,
      7.4);

    public static Translation2d redDepot = new Translation2d(
      15.54,
      2.1);

    public static Translation2d redHub = new Translation2d(
      11.91,
      4
    );

    public static Pose2d redHubPose = new Pose2d(redHub, new Rotation2d(0.0));
  }

  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2026RebuiltWelded.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}