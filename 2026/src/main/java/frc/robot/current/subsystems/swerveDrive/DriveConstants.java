// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.current.subsystems.swerveDrive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.current.Constants.SwerveConstants;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 6.858 * MathUtil.clamp(SwerveConstants.speedLimit, 0, 1);
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(14.75);      // This is the robot's Y, but we needed to switch with wheelbase in order for it to spin properly
  public static final double wheelBase = Units.inchesToMeters(29.75);       // This is the robot's X   ^^^^^^^^^^^^^^^^^^^^
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation =
      new Rotation2d(Units.degreesToRadians(99.63));
  public static final Rotation2d frontRightZeroRotation =
      new Rotation2d(Units.degreesToRadians(86.87));
  public static final Rotation2d backLeftZeroRotation =
      new Rotation2d(Units.degreesToRadians(-117.57));
  public static final Rotation2d backRightZeroRotation =
      new Rotation2d(Units.degreesToRadians(137.57));

  // Device CAN IDs
  public static final int pigeonCanId = 53; // changed to random since we aren't using it

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 5;
  public static final int frontRightDriveCanId = 3;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 6;
  public static final int frontRightTurnCanId = 4;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftTurnEncoderCanId = 9;
  public static final int backLeftTurnEncoderCanId = 11;
  public static final int frontRightTurnEncoderCanId = 10;
  public static final int backRightTurnEncoderCanId = 12;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = 0.038;
  public static final double driveMotorReduction = (5.27); // Mk5i R3 gearing
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.09903;
  public static final double driveKv = 0.08621;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 26;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // Rotor RPM -> Wheel Rad/Sec
  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
