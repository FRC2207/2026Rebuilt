package frc.robot.lib.ObjectVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;

public interface ObjectVisionIO {
  @AutoLog
  public static class objectVisionIOInputs {
    public List<Translation2d> fieldPositions;
    public boolean hopperSeesObject;
  }

  public default void updateInputs(objectVisionIOInputs inputs) {}

  public abstract Trajectory getPath();

  public abstract Boolean hopperSeesObject();
}
