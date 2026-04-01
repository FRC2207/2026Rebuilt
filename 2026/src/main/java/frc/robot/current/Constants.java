// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String fieldType = "welded";

  public static final boolean isTuningMode = false;

  // The period for the main robot loop. Used for simulation.
  // 20ms is the default for FRC robots, and matches the default period of the
  // CommandScheduler.
  public static final double loopPeriodSecs = 0.02; // 20ms

  /**
   * This is how the SysID knows which motor on the module to test.See the example
   * command to run a sysId test,
   * and see how the String is used to determine the SysID test
   * 
   * @see
   *      {@link frc.robot.lib.swerve.updated.SwerveDrive}
   *      {@link frc.robot.lib.swerve.updated.SwerveDrive#sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction)}
   *      {@link frc.robot.lib.swerve.updated.Module#runCharacterization()
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
    public static final double speedLimit = 0.75;
  }

  public static class IntakeConstants {
    public static final int intakeID = 41;
    public static final int followerID = 42;

    public static final double intakeSpeed = 1750;
    public static final double kSim_MOI = 0.0007; // kg*m^2, moment of inertia of the flywheel being simulated for the
                                                  // intake motor
    public static final double kSim_GearReduction = 1.0; // Gear reduction of the intake motor

    // PID
    public static final double kP = 0.00007;
    public static final double kI = 0;
    public static final double kD = 0.0003;
    public static final double kS = 0;
    public static final double kV = 0.00195;
    public static final double kA = 0;

    public static final double kSim_P = 0.006;
    public static final double kSim_I = 0.0;
    public static final double kSim_D = 0.0001;
    public static final double kSim_S = 0.0;
    public static final double kSim_V = 0.00175;
  }

  public static class PivotConstants {
    public static final int pivotID = 40;

    // PID
    public static final double kP = 1.9;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;
    public static final double kCos = 0.7;

    public static final double kSim_P = 12.6;
    public static final double kSim_I = 0;
    public static final double kSim_D = 18.9;

    // THESE VALUES ARE ROTATIONS AND NOT ANGLES, MAKE SURE THEY ARE SMALL
    public static final double storedRotations = 0.20;
    public static final double collectionRotations = 0.0;
    public static final double intermediateRotations = 0.08;
  }

  public static class HopperConstants {
    public static final int motorID = 32;
    public static final double motorSpeed = -0.67; // Motor Speed as a percentage
  }

  public static class OuttakeConstants {
    public static final int highMotorId = 31;
    public static final int lowMotorId = 30;

    public static final double kSim_TopMOI = 0.0007; // kg*m^2, moment of inertia of the flywheel being simulated for
                                                     // the top motor
    public static final double kSim_TopGearReduction = 1.0; // Gear reduction of the top motor
    public static final double kSim_BottomMOI = 0.0007; // kg*m^2, moment of inertia of the flywheel being simulated for
                                                        // the bottom motor
    public static final double kSim_BottomGearReduction = 1.0; // Gear reduction of the bottom motor

    // The velocity for quick launch and continous launch
    public static final double velocityDefault = 2600; //3267 - 1000; // 67!

    public static final double kP = 0.00008;
    public static final double kI = 0.0;
    public static final double kD = 0.02;
    public static final double kS = 0.0;
    public static final double kV = 0.001816;
    public static final double kA = 0.0;

    public static final double kSim_P = 0.006;
    public static final double kSim_I = 0.0;
    public static final double kSim_D = 0.0001;
    public static final double kSim_S = 0.0;
    public static final double kSim_V = 0.00175;
  }

  public static class ClimberConstants{
    public static final int leftClimbMotorID = 50;
    public static final int rightClimbMotorID = 51;

    // The speed of the motor when climbing as a percentage
    public static final double climbSpeed = 0.3;

    // The maximum and minimum positions for the left climber in rotations
    public static final double leftClimbMax = 0.5;
    public static final double leftClimbMin = 0.5;

    // The maximum and minimum positions for the right climber in rotations
    public static final double rightClimbMax = 0.5;
    public static final double rightClimbMin = 0.5;
  }

}
