package frc.robot.lib.ObjectVision;
import edu.wpi.first.math.geometry.Pose2d;

import org.littletonrobotics.junction.AutoLog;
import java.util.function.Consumer;

public interface ObjectVisionIO {
  @AutoLog
  public static class ObjectVisionIOInputs {
    public double[] fuelX = new double[]{};
    public double[] fuelY = new double[]{};
    public boolean hopperSeesObject;
    public Pose2d[] kindleWaypoints = new Pose2d[]{};
  }

  public default void updateInputs(ObjectVisionIOInputs inputs) {}

  public default void setWaypointListener(Consumer<Pose2d[]> listener) {}
}