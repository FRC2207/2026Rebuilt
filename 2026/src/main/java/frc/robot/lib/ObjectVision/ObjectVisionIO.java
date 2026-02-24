package frc.robot.lib.ObjectVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public interface ObjectVisionIO {
  @AutoLog
  public static class objectVisionIOInputs {
    public int ballsDetected = 0;
  }

  public default void updateInputs(objectVisionIOInputs inputs) {}

  public abstract Trajectory getPath();

  public abstract Pose2d[] subsystemData(String subsystemName);

  public abstract Boolean seesObject();
}
