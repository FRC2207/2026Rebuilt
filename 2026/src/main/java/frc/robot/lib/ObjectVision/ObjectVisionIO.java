package frc.robot.lib.ObjectVision;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectVisionIO {
  @AutoLog
  public static class ObjectVisionIOInputs {
    public double[] fuelX = new double[]{};
    public double[] fuelY = new double[]{};
    public boolean hopperSeesObject;
  }

  public default void updateInputs(ObjectVisionIOInputs inputs) {}
}