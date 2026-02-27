package frc.robot.lib.ObjectVision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface ObjectVisionIO {
  @AutoLog
  public static class objectVisionIOInputs {
    public List<Translation2d> fieldPositions;
    public boolean hopperSeesObject;
  }

  public default void updateInputs(objectVisionIOInputs inputs) {}

  public abstract Command getPath();
  public abstract Command getDynamicPath();

  public abstract Boolean hopperSeesObject();
}
