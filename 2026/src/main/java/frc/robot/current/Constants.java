// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String robot = "Real";
  public static final String fieldType = "welded";

  public static final boolean isTuningMode = true;

  /**
   * This is how the SysID knows which motor on the module to test.See the example command to run a sysId test,
   * and see how the String is used to determine the SysID test
   * 
   * @see
   * {@link frc.robot.lib.swerve.updated.SwerveDrive}
   * {@link frc.robot.lib.swerve.updated.SwerveDrive#sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction)}
   * {@link frc.robot.lib.swerve.updated.Module#runCharacterization()
   */
  public final static String moduleSysId = "rotation"; // Either rotation or drive

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOtherControllerPort = 1;
  }

  public static class SwerveConstants {
    public static final double speedLimit = 0.5;
  }

  public static class IntakeConstants {
    //TODO: Get Motor IDs
    public static final int motorID = 30;
    public static final int pivotMotorID = 100000;
    public static final double intakeSpeed = 0.5;

    // PID
    // TODO: Tune these values
    public static final double kP = 1.3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // TODO: Get these values
    public static final double storedAngle = 90;
    public static final double collectionAngle = 0;
  }
  public static class OuttakeConstants {
    public static final int highMotorId = 20;
    public static final int lowMotorId = 31;

    // The velocity for quick launch and continous launch
    public static final double velocityDefault = 2000;

    public static final double kP = 0.00006;
    public static final double kI = 0.0;
    public static final double kD = 0.0003;
    public static final double kS = 0.0;
    public static final double kV = 0.00181;
    public static final double kA = 0.0;
  }

}
